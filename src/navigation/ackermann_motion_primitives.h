#ifndef ACKERMANN_MOTION_PRIMITIVES_H
#define ACKERMANN_MOTION_PRIMITIVES_H

#include <cmath>
#include <memory>
#include <vector>

#include "constant_curvature_arc.h"
#include "eigen3/Eigen/Dense"
#include "motion_primitives.h"
#include "shared/math/math_util.h"
#include "navigation_params.h"

using std::fabs;
using std::max;
using std::min;
using math_util::Sq;
using math_util::Sign;
using math_util::Pow;

namespace motion_primitives {

// Samples constant-curvature ackermann arcs. Inherits the generic sampler
// interface (Update / GetSamples / SetNavParams and the shared state members
// vel, ang_vel, local_target, point_cloud, nav_params).
class AckermannSampler : public PathRolloutSamplerBase {
 public:
  AckermannSampler() = default;
  explicit AckermannSampler(const navigation::NavigationParams& params) {
    nav_params = params;
  }

  std::vector<std::shared_ptr<PathRolloutBase>> GetSamples(int n) override;

  void setPathLength(std::shared_ptr<ConstantCurvatureArc> path_ptr);
  virtual void checkObstacles(std::shared_ptr<ConstantCurvatureArc> path_ptr);
};

struct PathMetrics {
  float goal_dist;  // L2 distance from the path endpoint to the local subgoal
  float heading;    // residual angle (rad) between the endpoint heading and the
                    // bearing from the endpoint to the local subgoal (0 = aligned)
  float clearance;  // distance to nearest obstacle along the path
  float velocity;   // commanded forward velocity for the path
};

// DWA-style evaluator over ackermann arcs. Inherits the generic evaluator
// interface (Update / FindBest and the shared state members).
class AckermannEvaluator : public PathEvaluatorBase {
 public:
  explicit AckermannEvaluator(const navigation::NavigationParams& nav_params)
      : nav_params_(nav_params) {}
  ~AckermannEvaluator() override = default;

  std::shared_ptr<PathRolloutBase> FindBest(
      const std::vector<std::shared_ptr<PathRolloutBase>>& paths) override;

  PathMetrics computeMetrics(const std::shared_ptr<PathRolloutBase>& path_ptr);

 private:
  navigation::NavigationParams nav_params_;
};
}  // namespace motion_primitives

#endif  // ACKERMANN_MOTION_PRIMITIVES_H