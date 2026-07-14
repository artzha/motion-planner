#ifndef DIFFERENTIAL_MOTION_PRIMITIVES_H
#define DIFFERENTIAL_MOTION_PRIMITIVES_H

#include <memory>
#include <vector>

#include "ackermann_motion_primitives.h"
#include "constant_curvature_arc.h"
#include "eigen3/Eigen/Dense"

namespace motion_primitives {

// Differential-drive sampler. Reuses the ackermann arc samples and obstacle
// checking, and additionally offers pure turn-in-place options so the robot can
// rotate toward targets it cannot reach within its steering limits.
class DifferentialSampler : public AckermannSampler {
 public:
  using AckermannSampler::AckermannSampler;

  std::vector<std::shared_ptr<PathRolloutBase>> GetSamples(int n) override;
  void checkObstacles(std::shared_ptr<ConstantCurvatureArc> path_ptr) override;
};

}  // namespace motion_primitives

#endif  // DIFFERENTIAL_MOTION_PRIMITIVES_H
