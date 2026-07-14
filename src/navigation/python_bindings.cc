#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

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
  const auto buf = point_cloud.unchecked<2>();
  if (buf.shape(1) != 2) {
    throw std::invalid_argument("point_cloud must have shape (N, 2)");
  }
  std::vector<Eigen::Vector2f> cloud;
  cloud.reserve(buf.shape(0));
  for (py::ssize_t i = 0; i < buf.shape(0); ++i) {
    cloud.emplace_back(buf(i, 0), buf(i, 1));
  }

  const navigation::NavigationParams params = navigation::LoadConfig(config_path);

  const navigation::HybridPlanResult res = navigation::PlanHybridAStar(
      cloud, Eigen::Vector2f(goal.first, goal.second), params, world_size, seed,
      noise, weight);

  // Flatten the densified (x, y, theta, v, omega) trajectory into rows.
  std::vector<std::array<float, 5>> rows;
  rows.reserve(res.path.size());
  for (const auto& s : res.path) {
    rows.push_back({s.loc.x(), s.loc.y(), s.theta, s.v, s.omega});
  }

  py::array_t<float> out({static_cast<py::ssize_t>(rows.size()),
                          static_cast<py::ssize_t>(5)});
  auto o = out.mutable_unchecked<2>();
  for (size_t i = 0; i < rows.size(); ++i) {
    for (int j = 0; j < 5; ++j) o(i, j) = rows[i][j];
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
}
