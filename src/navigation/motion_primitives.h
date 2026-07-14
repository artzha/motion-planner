#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"

#include "math/poses_2d.h"
#include "navigation_params.h"
#include "shared/math/math_util.h"

namespace motion_primitives {

// A path rollout, starting at the robot's current pose (the origin, 0,0,0) in
// the robot/base_link frame.
struct PathRolloutBase {
  virtual ~PathRolloutBase() = default;

  // Cumulative translational distance traversed along the rollout,
  // \int ||v(t)|| dt.
  virtual float Length() const = 0;

  // Free path length: the collision-free distance along the rollout direction.
  virtual float FPL() const = 0;

  // Cumulative angular distance traversed, \int ||\dot{\theta}(t)|| dt.
  virtual float AngularLength() const = 0;

  // Pose of the robot at the end of the rollout (base_link frame).
  virtual pose_2d::Pose2Df EndPoint() const = 0;

  // Pose of the robot at fraction f in [0, 1] along the rollout (base_link).
  virtual pose_2d::Pose2Df GetIntermediateState(float f) const = 0;

  // Obstacle clearance along the rollout.
  virtual float Clearance() const = 0;

  // Actuation commands (linear + angular velocity) to execute this rollout over
  // one control period dt, given the current linear and angular velocity.
  virtual void GetControls(const navigation::MotionLimits& linear_limits,
                           const navigation::MotionLimits& angular_limits,
                           const float dt,
                           const Eigen::Vector2f& linear_vel,
                           const float angular_vel,
                           Eigen::Vector2f& vel_cmd,
                           float& ang_vel_cmd) const = 0;
};

// Samples collision-free path rollout options from the local dynamic state.
struct PathRolloutSamplerBase {
  virtual ~PathRolloutSamplerBase() = default;

  // Return up to n valid, collision-free rollout options.
  virtual std::vector<std::shared_ptr<PathRolloutBase>> GetSamples(int n) = 0;

  // Update the local navigation state (current velocity, local target, and the
  // obstacle point cloud).
  virtual void Update(const Eigen::Vector2f& new_vel,
                      const float new_ang_vel,
                      const Eigen::Vector2f& new_local_target,
                      const std::vector<Eigen::Vector2f>& new_point_cloud) {
    vel = new_vel;
    ang_vel = new_ang_vel;
    local_target = new_local_target;
    point_cloud = new_point_cloud;
  }

  void SetNavParams(const navigation::NavigationParams& new_params) {
    nav_params = new_params;
  }

  // Current linear velocity.
  Eigen::Vector2f vel = Eigen::Vector2f::Zero();
  // Current angular velocity.
  float ang_vel = 0.0f;
  // Local navigation target (base_link frame).
  Eigen::Vector2f local_target = Eigen::Vector2f::Zero();
  // Obstacle point cloud (base_link frame).
  std::vector<Eigen::Vector2f> point_cloud;
  // Navigation parameters.
  navigation::NavigationParams nav_params;
};

// Evaluates and selects the best rollout from a set of options.
struct PathEvaluatorBase {
  PathEvaluatorBase()
      : curr_loc(0, 0), curr_ang(0), vel(0, 0), ang_vel(0), local_target(0, 0) {}
  virtual ~PathEvaluatorBase() = default;

  // Update the local navigation state.
  virtual void Update(const Eigen::Vector2f& new_loc,
                      const float new_ang,
                      const Eigen::Vector2f& new_vel,
                      const float new_ang_vel,
                      const Eigen::Vector2f& new_local_target,
                      const std::vector<Eigen::Vector2f>& new_point_cloud) {
    curr_loc = new_loc;
    curr_ang = new_ang;
    vel = new_vel;
    ang_vel = new_ang_vel;
    local_target = new_local_target;
    point_cloud = new_point_cloud;
  }

  // Return the best rollout from the provided set of paths.
  virtual std::shared_ptr<PathRolloutBase> FindBest(
      const std::vector<std::shared_ptr<PathRolloutBase>>& paths) = 0;

  // Current location.
  Eigen::Vector2f curr_loc;
  // Current angle.
  float curr_ang;
  // Current linear velocity.
  Eigen::Vector2f vel;
  // Current angular velocity.
  float ang_vel;
  // Local navigation target (base_link frame).
  Eigen::Vector2f local_target;
  // Obstacle point cloud (base_link frame).
  std::vector<Eigen::Vector2f> point_cloud;
};

float run1DTOC(const navigation::MotionLimits &limits, const float x0,
               const float v0, const float xf, const float vf,
               const float dt = 0.05);
}  // namespace motion_primitives
