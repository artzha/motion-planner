#include "hybrid_planner.h"

#include "astar.h"

namespace navigation {

HybridPlanResult PlanHybridAStar(
    const std::vector<Eigen::Vector2f>& cloud_base_link,
    const Eigen::Vector2f& goal_base_link,
    const NavigationParams& params,
    float world_size,
    uint64_t seed,
    float noise,
    float weight) {
  HybridPlanResult result;

  const Eigen::Vector2f origin(-world_size / 2.0f, -world_size / 2.0f);
  const DifferentialDomain::State start(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  const DifferentialDomain::State goal_state(goal_base_link.x(),
                                             goal_base_link.y(), 0.0f, 0.0f,
                                             0.0f);
  DifferentialDomain domain(params, origin, world_size, goal_state.loc,
                            cloud_base_link, seed, noise);

  DifferentialDomain::NullVisualizer viz;
  std::vector<DifferentialDomain::State> path;
  const bool found = AStar(start, goal_state, domain, &viz, &path, weight);
  if (!found || path.empty()) return result;

  // Densify each edge by replaying its constant control through the same
  // integrator the planner used, yielding continuous (x, y, theta, v, omega).
  result.path.push_back(path.front());
  for (size_t i = 1; i < path.size(); ++i) {
    const std::vector<DifferentialDomain::State> roll =
        domain.Rollout(path[i - 1], path[i].v, path[i].omega, nullptr);
    for (const auto& s : roll) result.path.push_back(s);
  }
  result.found = true;
  return result;
}

}  // namespace navigation
