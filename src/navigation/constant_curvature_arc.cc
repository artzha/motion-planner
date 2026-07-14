#include "constant_curvature_arc.h"

#include <algorithm>

#include "motion_primitives.h"
#include "shared/math/math_util.h"

using std::fabs;
using math_util::Sign;

namespace {
const float kEpsilon = 1e-5;
}  // namespace

namespace motion_primitives {

float ConstantCurvatureArc::Length() const {
  // Turn-in-place rollouts traverse no translational distance.
  return is_turn_in_place() ? 0.0f : arc_length_;
}

float ConstantCurvatureArc::FPL() const { return Length(); }

float ConstantCurvatureArc::AngularLength() const {
  // For turn-in-place, arc_length_ stores the turn angle; otherwise the swept
  // angle is arc_length * curvature.
  return is_turn_in_place() ? fabs(arc_length_)
                            : fabs(arc_length_ * curvature_);
}

pose_2d::Pose2Df ConstantCurvatureArc::GetIntermediateState(float f) const {
  f = std::max(0.0f, std::min(1.0f, f));
  if (is_turn_in_place()) {
    // Rotate in place; the direction is the sign of the (infinite) curvature.
    const float angle = f * std::copysign(arc_length_, curvature_);
    return pose_2d::Pose2Df(angle, Eigen::Vector2f(0.0f, 0.0f));
  }
  const float s = f * arc_length_;
  if (fabs(curvature_) < kEpsilon) {
    return pose_2d::Pose2Df(0.0f, Eigen::Vector2f(s, 0.0f));
  }
  const float r = 1.0f / curvature_;
  const float theta = s * curvature_;
  return pose_2d::Pose2Df(
      theta, Eigen::Vector2f(r * std::sin(theta), r * (1.0f - std::cos(theta))));
}

pose_2d::Pose2Df ConstantCurvatureArc::EndPoint() const {
  return GetIntermediateState(1.0f);
}

void ConstantCurvatureArc::GetControls(
    const navigation::MotionLimits& linear_limits,
    const navigation::MotionLimits& angular_limits,
    const float dt,
    const Eigen::Vector2f& linear_vel,
    const float angular_vel,
    Eigen::Vector2f& vel_cmd,
    float& ang_vel_cmd) const {
  (void)angular_vel;
  if (is_turn_in_place()) {
    // Rotate in place toward the turn direction at the angular speed limit.
    vel_cmd = Eigen::Vector2f(0.0f, 0.0f);
    ang_vel_cmd = std::copysign(angular_limits.max_speed, curvature_);
    return;
  }
  const float v =
      run1DTOC(linear_limits, 0.0f, linear_vel.x(), arc_length_, 0.0f, dt);
  vel_cmd = Eigen::Vector2f(v, 0.0f);
  ang_vel_cmd = v * curvature_;  // omega = v * kappa
}

void ConstantCurvatureArc::getControlOnCurve(
    const navigation::MotionLimits& linear_limits,
    const float linear_vel,
    const float dt,
    float& cmd_linear_vel) const {
  if (is_turn_in_place()) {
    cmd_linear_vel = 0;
    return;
  }

  cmd_linear_vel =
      run1DTOC(linear_limits, 0, linear_vel, arc_length_, cmd_linear_vel, dt);
}

Eigen::Vector2f ConstantCurvatureArc::getEndPoint() const {
  /**
   * @brief Computes the xy endpoint of the arc in the local base_link frame
   *
   */
  if (is_turn_in_place()) {
    return Eigen::Vector2f(0, 0);
  }

  if (std::abs(curvature_) < kEpsilon) {
    return Eigen::Vector2f(arc_length_, 0);
  }

  const float r = 1 / curvature_;
  const float theta = arc_length_ * curvature_;
  return Eigen::Vector2f(r * std::sin(theta), r * (1 - std::cos(theta)));
}
}  // namespace motion_primitives
