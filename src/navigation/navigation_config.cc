#include "navigation_config.h"

#include "config_reader/config_reader.h"

namespace navigation {

NavigationParams LoadConfig(const std::string& lua_path) {
  NavigationParams params;

#define REAL_PARAM(x) CONFIG_DOUBLE(x, "NavigationParameters." #x);
  REAL_PARAM(dt);
  REAL_PARAM(system_latency);
  REAL_PARAM(max_linear_accel);
  REAL_PARAM(max_linear_deccel);
  REAL_PARAM(max_linear_speed);
  REAL_PARAM(max_angular_accel);
  REAL_PARAM(max_angular_deccel);
  REAL_PARAM(max_angular_speed);
  REAL_PARAM(max_curvature);
  REAL_PARAM(max_path_length);
  REAL_PARAM(max_clearance);
  REAL_PARAM(clearance_weight);
  REAL_PARAM(velocity_weight);
  REAL_PARAM(distance_weight);
  REAL_PARAM(goal_tolerance);
  REAL_PARAM(robot_length);
  REAL_PARAM(robot_width);
  REAL_PARAM(robot_wheelbase);
  REAL_PARAM(obstacle_margin);
  REAL_PARAM(lidar_offset);

#define GRID_PARAM(x) CONFIG_DOUBLE(x, "GridParameters." #x);
  GRID_PARAM(resolution);
  GRID_PARAM(half_extent);

  config_reader::ConfigReader reader({lua_path});
  params.dt = CONFIG_dt;
  params.system_latency = CONFIG_system_latency;
  params.linear_limits = navigation::MotionLimits(
      CONFIG_max_linear_accel, CONFIG_max_linear_deccel, CONFIG_max_linear_speed);
  params.angular_limits = navigation::MotionLimits(
      CONFIG_max_angular_accel, CONFIG_max_angular_deccel,
      CONFIG_max_angular_speed);
  params.max_curvature = CONFIG_max_curvature;
  params.max_path_length = CONFIG_max_path_length;
  params.max_clearance = CONFIG_max_clearance;
  params.clearance_weight = CONFIG_clearance_weight;
  params.velocity_weight = CONFIG_velocity_weight;
  params.distance_weight = CONFIG_distance_weight;
  params.goal_tolerance = CONFIG_goal_tolerance;
  params.robot_length = CONFIG_robot_length;
  params.robot_width = CONFIG_robot_width;
  params.robot_wheelbase = CONFIG_robot_wheelbase;
  params.obstacle_margin = CONFIG_obstacle_margin;
  params.lidar_offset = CONFIG_lidar_offset;
  params.grid.resolution = CONFIG_resolution;
  params.grid.half_extent = CONFIG_half_extent;
  return params;
}

}  // namespace navigation
