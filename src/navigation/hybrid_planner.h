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
// input cloud/goal (base_link, robot at the origin facing +x).
struct HybridPlanResult {
  bool found = false;
  std::vector<DifferentialDomain::State> path;
};

// Plans a differential-drive kinodynamic hybrid A* path from rest at the origin
// (0, 0, 0) to `goal_base_link`, avoiding `cloud_base_link` (base_link frame).
// `world_size` is the side length (m) of the square planning window centered on
// the origin. `noise` (eta, meters) adds a smooth seeded cost-map potential so
// paths diversify in corridors; `weight` (>= 1) is the weighted-A* heuristic
// factor. Each call is self-contained (no shared mutable state), so it is safe
// to run from multiple threads concurrently.
HybridPlanResult PlanHybridAStar(
    const std::vector<Eigen::Vector2f>& cloud_base_link,
    const Eigen::Vector2f& goal_base_link,
    const NavigationParams& params,
    float world_size,
    uint64_t seed = 0,
    float noise = 0.0f,
    float weight = 1.0f);

}  // namespace navigation

#endif  // MOTION_PLANNER_HYBRID_PLANNER_H
