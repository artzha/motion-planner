#ifndef CONSTANT_CURVATURE_ARC_H
#define CONSTANT_CURVATURE_ARC_H

#include <cmath>
#include <memory>

#include "eigen3/Eigen/Dense"

#include "math/poses_2d.h"
#include "motion_primitives.h"

namespace motion_primitives {
// A constant-curvature arc (or, when the curvature is infinite, a turn in
// place). Implements the generic PathRolloutBase interface, and additionally
// exposes concrete curvature/arc-length accessors used by the ackermann sampler
// and evaluator (and for visualization via down-casting).
class ConstantCurvatureArc : public PathRolloutBase {
 public:
  ConstantCurvatureArc() : curvature_(0), arc_length_(0), clearance_(0) {}
  ConstantCurvatureArc(float curvature)
      : curvature_(curvature), arc_length_(0), clearance_(0) {}
  ConstantCurvatureArc(float curvature, float arc_length)
      : curvature_(curvature), arc_length_(arc_length), clearance_(0) {}
  ConstantCurvatureArc(float curvature, float arc_length, float clearance)
      : curvature_(curvature), arc_length_(arc_length), clearance_(clearance) {}

  // ---- PathRolloutBase interface ----
  float Length() const override;
  float FPL() const override;
  float AngularLength() const override;
  pose_2d::Pose2Df EndPoint() const override;
  pose_2d::Pose2Df GetIntermediateState(float f) const override;
  float Clearance() const override { return clearance_; }
  void GetControls(const navigation::MotionLimits& linear_limits,
                   const navigation::MotionLimits& angular_limits,
                   const float dt,
                   const Eigen::Vector2f& linear_vel,
                   const float angular_vel,
                   Eigen::Vector2f& vel_cmd,
                   float& ang_vel_cmd) const override;

  // ---- Concrete accessors ----
  void getControlOnCurve(const navigation::MotionLimits& linear_limits,
                         const float linear_vel,
                         const float dt,
                         float& cmd_linear_vel) const;

  void set_arc_length(float arc_length) { arc_length_ = arc_length; }

  void set_clearance(float clearance) { clearance_ = clearance; }

  Eigen::Vector2f getEndPoint() const;

  float curvature() const { return curvature_; }

  float arc_length() const { return arc_length_; }

  float clearance() const { return clearance_; }

  bool is_turn_in_place() const { return std::isinf(curvature_); }

 private:
  float curvature_;
  float arc_length_;
  float clearance_;
};
}  // namespace motion_primitives

#endif  // CONSTANT_CURVATURE_ARC_H