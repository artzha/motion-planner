#ifndef MOTION_PLANNER_HYBRID_PLANNER_H
#define MOTION_PLANNER_HYBRID_PLANNER_H

#include <cstdint>
#include <vector>

#include "eigen3/Eigen/Dense"

#include "domain.h"
#include "navigation_params.h"

namespace navigation {

// Result of a single hybrid A* query. `path` is the densified trajectory
// (columns per state: loc.x, loc.y, theta, v, omega) in the same frame as the
// input cloud/goal (base_link, robot at the origin facing +x). `cost` is its
// arc-length-plus-turn cost in meters, excluding any diversity term, so costs
// from differently-penalized searches are directly comparable. `elapsed_s` times
// the search, which is worth having per-path because a round run under a cost
// bound prunes hard and finishes well ahead of an unbounded one.
struct HybridPlanResult {
  bool found = false;
  std::vector<DifferentialDomain::State> path;
  float cost = 0.0f;
  double elapsed_s = 0.0;
};

// Plans a differential-drive kinodynamic hybrid A* path from rest at the origin
// (0, 0, 0) to `goal_base_link`, avoiding `cloud_base_link` (base_link frame).
// `world_size` is the side length (m) of the square planning window centered on
// the origin. `weight` (>= 1) is the weighted-A* heuristic factor. `diversity`
// optionally biases the search away from other candidates, and `cost_bound`
// (> 0) caps the returned path's diversity-free cost. Each call is
// self-contained (no shared mutable state), so it is safe to run from multiple
// threads concurrently.
HybridPlanResult PlanHybridAStar(
    const std::vector<Eigen::Vector2f>& cloud_base_link,
    const Eigen::Vector2f& goal_base_link,
    const NavigationParams& params,
    float world_size,
    float weight = 1.0f,
    const DiversityParams& diversity = {},
    float cost_bound = 0.0f);

// Knobs for PlanDiversePaths.
struct DiversePlanOptions {
  int num_paths = 3;
  DiversityMode mode = DiversityMode::kBallPenalty;

  // Weighted-A* factor. Defaults above 1 because any diversity term inflates
  // edge costs, which flattens the f-ordering and makes the search expand far
  // more nodes; at w = 1 it routinely runs into the expansion cap and returns
  // nothing. Measured on the demo scene, w = 1.0 yielded 1-2 paths of 5 where
  // w = 1.3 yielded all 5.
  float weight = 1.3f;

  // The seeded cost-map potential. Under kNoise this is the whole mechanism and
  // path j is drawn with seed + j. Under kBallPenalty it is applied to the
  // detour rounds only (round 0 stays the exact optimum) and is what makes two
  // calls on the same scene return different sets; the balls alone are a
  // function of the accepted paths, so they answer identically every time.
  // Defaults off, so a ball-penalty search is reproducible unless asked
  // otherwise.
  uint64_t seed = 0;
  float noise = 0.0f;

  // kBallPenalty: geometry of the repulsion field laid along accepted paths.
  // ball_radius is its decay length; ball_weight the fraction by which it
  // inflates distance cost right on an accepted path, making it a rate rather
  // than a per-edge charge.
  //
  // ball_radius should stay under the half-width of the tightest gap the robot
  // must use: much past that the field covers a corridor entirely, no route
  // avoids it, and every path after the first is a near-duplicate. ball_weight
  // is how hard paths are pushed apart, and returns diminish sharply -- 2.0 and
  // 3.0 yield the same sets, the latter several times slower.
  float ball_radius = 0.6f;
  float ball_weight = 2.0f;
  float ball_spacing = 0.3f;

  // Radii around the shared start and the shared goal in which no ball is laid.
  // Both candidates leave the same pose and reach the same goal, so a ball there
  // charges every later round the same toll whichever way it went, and at the
  // goal end can wall the goal off so the remaining rounds fail outright.
  //
  // They are separate because they buy opposite things, and which one matters
  // depends on the scene. Shrinking the start zone pushes the rounds apart
  // sooner, so the set differs over the stretch the robot actually executes
  // before the next replan; widening the goal zone lets those rounds merge again
  // on the approach, which is what makes a second candidate reachable at all
  // when the goal sits in a bottleneck with only one way in.
  //
  // Negative means "use ball_radius", which is the behavior these replaced.
  float start_exclusion = -1.0f;
  float goal_exclusion = -1.0f;

  // Length budget for the detours, as a fraction over the optimal path's cost: a
  // path is only accepted under (1 + suboptimality) * optimal. Negative disables
  // the bound.
  //
  // Deliberately loose. Whether a longer route is a worse one is usually not
  // something this planner can see -- that judgement belongs to whatever scores
  // the candidates downstream -- so a tight budget mostly discards the detours
  // the repulsion just worked to find. It is not free either: the pruning is
  // work, and tightening it measurably slowed the search as well as thinning it.
  float suboptimality = 1.0f;

  // How far (m) a path must depart from every path already accepted to count as
  // a new one; the loop stops rather than return anything closer. Kept separate
  // from ball_radius, which sets how hard the search is pushed: tying "different
  // enough" to the strength of the push means a firmer push also silently raises
  // the bar, and the yield falls off a cliff.
  //
  // This is the dial for how fine-grained the set is. Low fills the set even
  // where routes are scarce, at a closer worst pair; high guarantees a floor and
  // gives up paths precisely in the tight scenes that have fewest to spare.
  float min_separation = 0.3f;
};

// Plans up to `num_paths` distinct paths to a single goal, in round order:
// index 0 is the unpenalized optimum and later entries detour progressively
// further to stay clear of the earlier ones. Returns fewer than requested (or
// empty) when the free space runs out.
//
// Under kBallPenalty each round repels from every path already accepted, so
// separation is enforced rather than left to chance -- unlike kNoise, which
// draws independently and therefore clusters. kNoise is retained so the two can
// be compared on the same scene; there it is simply `num_paths` independent
// seeded searches.
//
// Rounds are sequential by construction (each needs its predecessors), so this
// does no threading of its own; parallelize across goals instead.
std::vector<HybridPlanResult> PlanDiversePaths(
    const std::vector<Eigen::Vector2f>& cloud_base_link,
    const Eigen::Vector2f& goal_base_link,
    const NavigationParams& params,
    float world_size,
    const DiversePlanOptions& opts);

}  // namespace navigation

#endif  // MOTION_PLANNER_HYBRID_PLANNER_H
