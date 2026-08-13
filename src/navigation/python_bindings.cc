#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "eigen3/Eigen/Dense"

#include "hybrid_planner.h"
#include "navigation_config.h"
#include "navigation_params.h"

namespace py = pybind11;

namespace {

std::vector<Eigen::Vector2f> ToCloud(
    const py::array_t<float, py::array::c_style | py::array::forcecast>&
        point_cloud) {
  const auto buf = point_cloud.unchecked<2>();
  if (buf.shape(1) != 2) {
    throw std::invalid_argument("point_cloud must have shape (N, 2)");
  }
  std::vector<Eigen::Vector2f> cloud;
  cloud.reserve(buf.shape(0));
  for (py::ssize_t i = 0; i < buf.shape(0); ++i) {
    cloud.emplace_back(buf(i, 0), buf(i, 1));
  }
  return cloud;
}

// Flattens a densified trajectory into an (N, 5) array: x, y, theta, v, omega.
py::array_t<float> ToArray(
    const std::vector<navigation::DifferentialDomain::State>& path) {
  py::array_t<float> out({static_cast<py::ssize_t>(path.size()),
                          static_cast<py::ssize_t>(5)});
  auto o = out.mutable_unchecked<2>();
  for (size_t i = 0; i < path.size(); ++i) {
    const auto& s = path[i];
    o(i, 0) = s.loc.x();
    o(i, 1) = s.loc.y();
    o(i, 2) = s.theta;
    o(i, 3) = s.v;
    o(i, 4) = s.omega;
  }
  return out;
}

navigation::DiversityMode ParseMode(const std::string& mode) {
  if (mode == "none") return navigation::DiversityMode::kNone;
  if (mode == "noise") return navigation::DiversityMode::kNoise;
  if (mode == "ball") return navigation::DiversityMode::kBallPenalty;
  throw std::invalid_argument(
      "mode must be one of 'none', 'noise', 'ball'; got '" + mode + "'");
}

// Plans a differential-drive hybrid A* path from rest at the origin to `goal`,
// avoiding the base_link `point_cloud`. Returns an (N, 5) array of the densified
// trajectory: columns [x, y, theta, v, omega].
//
// `noise` (eta, meters) adds a smooth seeded cost-map potential so paths
// diversify in corridors; `weight` (>= 1) is the weighted-A* heuristic factor
// (greedier/faster). Both default to off (noise=0, weight=1) and are
// reproducible given `seed`.
py::array_t<float> plan(
    py::array_t<float, py::array::c_style | py::array::forcecast> point_cloud,
    std::pair<float, float> goal,
    const std::string& config_path,
    float world_size,
    uint64_t seed,
    float noise,
    float weight) {
  const std::vector<Eigen::Vector2f> cloud = ToCloud(point_cloud);
  const navigation::NavigationParams params = navigation::LoadConfig(config_path);

  navigation::DiversityParams div;
  div.mode = (noise > 0.0f) ? navigation::DiversityMode::kNoise
                            : navigation::DiversityMode::kNone;
  div.seed = seed;
  div.noise_eta = noise;

  const navigation::HybridPlanResult res = navigation::PlanHybridAStar(
      cloud, Eigen::Vector2f(goal.first, goal.second), params, world_size,
      weight, div);
  return ToArray(res.path);
}

// Plans up to `num_paths` distinct paths to one `goal`. Returns a list of dicts,
// one per path found, in round order (index 0 is the unpenalized optimum):
//   path       (N, 5) densified trajectory
//   cost       diversity-free arc-length + turn cost, meters
//   ratio      cost relative to path 0, i.e. the realized suboptimality
//   elapsed_s  wall time for that path's search
py::list plan_diverse(
    py::array_t<float, py::array::c_style | py::array::forcecast> point_cloud,
    std::pair<float, float> goal,
    const std::string& config_path,
    float world_size,
    int num_paths,
    const std::string& mode,
    float weight,
    uint64_t seed,
    float noise,
    float ball_radius,
    float ball_weight,
    float ball_spacing,
    float suboptimality,
    float min_separation) {
  const std::vector<Eigen::Vector2f> cloud = ToCloud(point_cloud);
  const navigation::NavigationParams params = navigation::LoadConfig(config_path);

  navigation::DiversePlanOptions opts;
  opts.num_paths = num_paths;
  opts.mode = ParseMode(mode);
  opts.weight = weight;
  opts.seed = seed;
  opts.noise = noise;
  opts.ball_radius = ball_radius;
  opts.ball_weight = ball_weight;
  opts.ball_spacing = ball_spacing;
  opts.suboptimality = suboptimality;
  opts.min_separation = min_separation;

  std::vector<navigation::HybridPlanResult> results;
  {
    py::gil_scoped_release release;
    results = navigation::PlanDiversePaths(
        cloud, Eigen::Vector2f(goal.first, goal.second), params, world_size,
        opts);
  }

  // Ratios are taken against the cheapest path found rather than against path 0,
  // so the number means "how much longer than the best route" in both modes.
  // Under 'ball' the two coincide, since path 0 is the unpenalized optimum.
  py::list out;
  float base = 1e-6f;
  if (!results.empty()) {
    const auto cheapest = std::min_element(
        results.begin(), results.end(),
        [](const navigation::HybridPlanResult& a,
           const navigation::HybridPlanResult& b) { return a.cost < b.cost; });
    base = std::max(cheapest->cost, 1e-6f);
  }
  for (const auto& res : results) {
    py::dict entry;
    entry["path"] = ToArray(res.path);
    entry["cost"] = res.cost;
    entry["ratio"] = res.cost / base;
    entry["elapsed_s"] = res.elapsed_s;
    out.append(std::move(entry));
  }
  return out;
}

}  // namespace

PYBIND11_MODULE(hybrid_astar, m) {
  m.doc() =
      "Differential-drive kinodynamic hybrid A* planner. plan() returns an "
      "(N, 5) array of x, y, theta, v, omega.";
  m.def("plan", &plan, py::arg("point_cloud"), py::arg("goal"),
        py::arg("config_path") = "config/navigation.lua",
        py::arg("world_size") = 12.8f,
        py::arg("seed") = 0,
        py::arg("noise") = 0.0f,
        py::arg("weight") = 1.0f,
        "Plan from rest at the origin to goal=(x, y) around a 2D point cloud "
        "(Nx2, base_link frame). seed/noise add a smooth cost-map potential "
        "(noise=eta in meters, 0=off); weight>=1 is the weighted-A* heuristic "
        "factor (1=optimal). Returns (N, 5): x, y, theta, v, omega.");

  m.def("plan_diverse", &plan_diverse, py::arg("point_cloud"), py::arg("goal"),
        py::arg("config_path") = "config/navigation.lua",
        py::arg("world_size") = 12.8f,
        py::arg("num_paths") = 3,
        py::arg("mode") = "ball",
        py::arg("weight") = 1.3f,
        py::arg("seed") = 0,
        py::arg("noise") = 0.0f,
        py::arg("ball_radius") = 0.6f,
        py::arg("ball_weight") = 2.0f,
        py::arg("ball_spacing") = 0.3f,
        py::arg("suboptimality") = 1.0f,
        py::arg("min_separation") = 0.3f,
        "Plan up to num_paths distinct paths to one goal=(x, y) around a 2D "
        "point cloud (Nx2, base_link frame).\n\n"
        "mode='ball' repels each round from the paths already accepted, so the "
        "set is separated by construction: every path returned departs from all "
        "the earlier ones by at least min_separation meters somewhere along its "
        "length, and every detour is capped at (1 + suboptimality) times the "
        "optimal cost, of which index 0 is that optimum. Under 'ball' the balls "
        "are a function of the accepted paths, so the set is reproducible; pass "
        "noise>0 to perturb the detour rounds (index 0 stays exact) and have "
        "repeated calls on the same scene return different sets. "
        "mode='noise' instead draws num_paths independent seeded "
        "searches (the pre-repulsion behavior), which is the baseline to "
        "compare against. mode='none' is deterministic, so it yields one "
        "distinct path.\n\n"
        "Returns a list of dicts, one per path found, in round order: 'path' "
        "(N, 5) as in plan(), 'cost' (diversity-free arc-length + turn cost, "
        "m), 'ratio' (cost over the cheapest path found), 'elapsed_s'. Fewer "
        "than num_paths entries means the free space ran out.");
}
