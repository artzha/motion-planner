#include "hybrid_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "astar.h"

namespace navigation {

namespace {

// Lays repulsion ball centers along `path`, roughly `spacing` apart, appending
// them to `balls`.
//
// The shared endpoints are skipped: every candidate leaves the same start pose
// and, within one PlanDiversePaths call, reaches the same goal. Balls there
// would charge every later round the same toll no matter which way it went
// (buying no separation) and, at the goal end, can wall the goal off so the
// remaining rounds all fail.
void AppendBalls(const std::vector<DifferentialDomain::State>& path,
                 const Eigen::Vector2f& goal,
                 const DiversePlanOptions& opts,
                 std::vector<Eigen::Vector2f>* balls) {
  const float spacing = std::max(opts.ball_spacing, 1e-3f);
  // Start ready to emit, so the first eligible point past the exclusion zone
  // gets a ball.
  float since_last = spacing;
  for (size_t i = 1; i < path.size(); ++i) {
    since_last += (path[i].loc - path[i - 1].loc).norm();
    if (since_last < spacing) continue;
    const Eigen::Vector2f& p = path[i].loc;
    // Note the accumulator is deliberately not reset when a point is skipped
    // for being in an exclusion zone, so emission resumes at the right spacing
    // as soon as the path clears it.
    if (p.norm() < opts.ball_radius) continue;
    if ((p - goal).norm() < opts.ball_radius) continue;
    balls->push_back(p);
    since_last = 0.0f;
  }
}

// Farthest any point of `path` strays from the closest point of any already
// accepted path -- the directed Hausdorff distance from the candidate to their
// union. Near zero means the candidate retraced one of them.
//
// Measured against the paths themselves rather than against the repulsion ball
// centers: those centers are absent near the shared start and goal, so distance
// to them reads as large wherever a candidate happens to differ near an
// endpoint, which would pass a candidate that is identical through the middle
// -- exactly the case worth rejecting.
float MaxDeviation(const std::vector<DifferentialDomain::State>& path,
                   const std::vector<HybridPlanResult>& accepted) {
  if (accepted.empty()) return std::numeric_limits<float>::max();
  float worst = 0.0f;
  for (const auto& s : path) {
    float nearest = std::numeric_limits<float>::max();
    for (const auto& other : accepted) {
      for (const auto& o : other.path) {
        nearest = std::min(nearest, (s.loc - o.loc).squaredNorm());
      }
    }
    worst = std::max(worst, nearest);
  }
  return std::sqrt(worst);
}

}  // namespace

HybridPlanResult PlanHybridAStar(
    const std::vector<Eigen::Vector2f>& cloud_base_link,
    const Eigen::Vector2f& goal_base_link,
    const NavigationParams& params,
    float world_size,
    float weight,
    const DiversityParams& diversity,
    float cost_bound) {
  HybridPlanResult result;

  const Eigen::Vector2f origin(-world_size / 2.0f, -world_size / 2.0f);
  const DifferentialDomain::State start(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  const DifferentialDomain::State goal_state(goal_base_link.x(),
                                             goal_base_link.y(), 0.0f, 0.0f,
                                             0.0f);
  DifferentialDomain domain(params, origin, world_size, goal_state.loc,
                            cloud_base_link, diversity);

  DifferentialDomain::NullVisualizer viz;
  std::vector<DifferentialDomain::State> path;
  float cost = 0.0f;
  const double t_start = GetMonotonicTime();
  const bool found = AStar(start, goal_state, domain, &viz, &path, weight,
                           cost_bound, &cost);
  result.elapsed_s = GetMonotonicTime() - t_start;
  if (!found || path.empty()) return result;

  // Densify each edge by replaying its constant control through the same
  // integrator the planner used, yielding continuous (x, y, theta, v, omega).
  result.path.push_back(path.front());
  for (size_t i = 1; i < path.size(); ++i) {
    const std::vector<DifferentialDomain::State> roll =
        domain.Rollout(path[i - 1], path[i].v, path[i].omega, nullptr);
    for (const auto& s : roll) result.path.push_back(s);
  }
  result.cost = cost;
  result.found = true;
  return result;
}

std::vector<HybridPlanResult> PlanDiversePaths(
    const std::vector<Eigen::Vector2f>& cloud_base_link,
    const Eigen::Vector2f& goal_base_link,
    const NavigationParams& params,
    float world_size,
    const DiversePlanOptions& opts) {
  std::vector<HybridPlanResult> out;
  if (opts.num_paths <= 0) return out;

  // Independent draws: no state carries between paths, so this reproduces the
  // pre-repulsion behavior exactly and serves as the baseline to compare against.
  // Only successes are returned, so a short result means the same thing in both
  // modes -- that many of the requested paths were found.
  if (opts.mode != DiversityMode::kBallPenalty) {
    out.reserve(opts.num_paths);
    for (int j = 0; j < opts.num_paths; ++j) {
      DiversityParams div;
      div.mode = opts.mode;
      div.seed = opts.seed + static_cast<uint64_t>(j);
      div.noise_eta = opts.noise;
      HybridPlanResult res = PlanHybridAStar(cloud_base_link, goal_base_link,
                                             params, world_size, opts.weight,
                                             div);
      if (res.found) out.push_back(std::move(res));
    }
    return out;
  }

  DiversityParams div;
  div.mode = DiversityMode::kBallPenalty;
  div.ball_radius = opts.ball_radius;
  div.ball_weight = opts.ball_weight;

  // Round 0 carries no balls, so BallPenalty returns 0 throughout and this is
  // just the optimal path. Its cost sets the budget every later round detours
  // within, so it is also unbounded -- there is nothing to be relative to yet.
  HybridPlanResult first = PlanHybridAStar(cloud_base_link, goal_base_link,
                                           params, world_size, opts.weight, div);
  if (!first.found) return out;
  const float cost_bound = (opts.suboptimality >= 0.0f)
                               ? (1.0f + opts.suboptimality) * first.cost
                               : 0.0f;
  AppendBalls(first.path, goal_base_link, opts, &div.balls);
  out.push_back(std::move(first));

  // Arm the noise field only now, so it perturbs the detours but never round 0.
  // Two things depend on round 0 staying unjittered: it is the path the robot
  // actually wants absent any other consideration, and its cost is the reference
  // for cost_bound, so keeping it exact means `suboptimality` stays "within this
  // fraction of the true optimum" instead of drifting with each draw.
  //
  // One field for all the detour rounds, not one per round: separation within a
  // single call is the balls' job, and the noise is here purely so that repeated
  // calls on an unchanged cloud and goal do not return an identical set.
  div.seed = opts.seed;
  div.noise_eta = opts.noise;

  while (static_cast<int>(out.size()) < opts.num_paths) {
    HybridPlanResult res =
        PlanHybridAStar(cloud_base_link, goal_base_link, params, world_size,
                        opts.weight, div, cost_bound);
    if (!res.found) {
      // Nothing clears the accepted paths within budget. Ease off once and
      // retry: a weaker penalty still prefers a new lane where one exists, and
      // scaling the weight needs no rework, whereas shrinking the radius would
      // mean re-deriving every ball. The reduction is scoped to this round --
      // left in place it compounds, and the repulsion decays until later rounds
      // just retrace earlier ones.
      const float full_weight = div.ball_weight;
      div.ball_weight *= 0.5f;
      res = PlanHybridAStar(cloud_base_link, goal_base_link, params, world_size,
                            opts.weight, div, cost_bound);
      div.ball_weight = full_weight;
      if (!res.found) break;
    }
    // A path that never leaves the corridor the accepted ones occupy is a
    // near-duplicate, and returning it defeats the point: downstream each
    // candidate costs a VLM evaluation, so a redundant one is worse than a
    // short list. Stop rather than pad: laying this path's balls and trying
    // again does not help, measurably -- crowding the field shuffles the next
    // round within the same corridor rather than pushing it clear, and costs a
    // search each time.
    if (MaxDeviation(res.path, out) < opts.min_separation) break;
    AppendBalls(res.path, goal_base_link, opts, &div.balls);
    out.push_back(std::move(res));
  }
  return out;
}

}  // namespace navigation
