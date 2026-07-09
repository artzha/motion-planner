


#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <gflags/gflags.h>

#include "astar.h"
#include "domain.h"
#include "eigen3/Eigen/Dense"
#include "navigation_config.h"
#include "navigation_params.h"

DEFINE_string(nav_config, "config/navigation.lua",
              "Path to the navigation config lua file.");
DEFINE_bool(test_avoidance, false, "Run obstacle avoidance test");
DEFINE_string(path_output, "astar_path.csv",
              "File to write the planned (x, y, theta) path to.");

void SignalHandler(int) {
  printf("Exiting.\n");
  exit(0);
}

// Densifies the hybrid-A* state sequence into a smooth sequence of continuous
// (x, y, theta) poses by replaying each edge's constant control (the successor's
// v/omega) through the same unicycle integrator the planner used. `path` is
// ordered start -> goal.
std::vector<Eigen::Vector3f> DensifyPath(
    const navigation::DifferentialDomain& domain,
    const std::vector<navigation::DifferentialDomain::State>& path) {
  std::vector<Eigen::Vector3f> poses;
  if (path.empty()) return poses;

  auto wrap = [](float a) { return std::atan2(std::sin(a), std::cos(a)); };

  const auto& p0 = path.front();
  poses.emplace_back(p0.loc.x(), p0.loc.y(), wrap(p0.theta));

  for (size_t i = 1; i < path.size(); ++i) {
    const auto& a = path[i - 1];
    const auto& b = path[i];
    // The control that produced b is carried in b's velocity fields.
    const std::vector<navigation::DifferentialDomain::State> roll =
        domain.Rollout(a, b.v, b.omega, nullptr);
    for (const auto& s : roll) {
      poses.emplace_back(s.loc.x(), s.loc.y(), wrap(s.theta));
    }
  }
  return poses;
}

void WritePath(const std::string& filename,
               const std::vector<Eigen::Vector3f>& poses) {
  std::ofstream out(filename);
  if (!out) {
    printf("Failed to open %s for writing.\n", filename.c_str());
    return;
  }
  out << "x,y,theta\n";
  for (const auto& p : poses) {
    out << p.x() << "," << p.y() << "," << p.z() << "\n";
  }
  printf("Wrote %zu poses to %s\n", poses.size(), filename.c_str());
}

// Renders a top-down ASCII view of the scene in base_link (+x up, +y left).
void DrawScene(const std::vector<Eigen::Vector2f>& point_cloud,
               const Eigen::Vector2f& start,
               const Eigen::Vector2f& goal,
               const std::vector<Eigen::Vector2f>& path) {
  const float extent = 4.0f;  // 8m x 8m ASCII view centered on base_link
  const float res = 0.2f;
  const int n = static_cast<int>(2 * extent / res) + 1;
  std::vector<std::string> grid(n, std::string(n, '.'));

  auto put = [&](const Eigen::Vector2f& p, char ch) {
    const int row = static_cast<int>(std::lround((extent - p.x()) / res));
    const int col = static_cast<int>(std::lround((extent - p.y()) / res));
    if (row >= 0 && row < n && col >= 0 && col < n) grid[row][col] = ch;
  };

  for (const auto& p : path) put(p, '*');
  for (const auto& p : point_cloud) put(p, '#');
  put(goal, 'G');
  put(start, 'R');

  std::cout << "\nTop-down view (base_link): +x up, +y left\n"
            << "  R = robot, G = goal, # = obstacle, * = A* path\n\n";
  for (const auto& row : grid) std::cout << "  " << row << "\n";
  std::cout << std::endl;
}

void TestAvoidance(const navigation::NavigationParams& params) {
  printf("Testing hybrid A* avoidance on a differential-drive robot...\n");

  // Local point cloud (base_link): a wall blocking the straight path to the
  // goal, forcing the planner to route around it.
  std::vector<Eigen::Vector2f> point_cloud;
  for (float y = -0.8f; y <= 0.8f + 1e-3f; y += 0.1f) {
    point_cloud.emplace_back(1.5f, y);
  }

  // A 12.8m x 12.8m world at 0.1m/cell resolution, centered on base_link.
  const float kWorldSize = 12.8f;
  const Eigen::Vector2f origin(-kWorldSize / 2.0f, -kWorldSize / 2.0f);

  // Start from rest facing +x; goal 3m ahead (heading/velocity there are free).
  const navigation::DifferentialDomain::State start(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  const navigation::DifferentialDomain::State goal(3.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  navigation::DifferentialDomain domain(params, origin, kWorldSize, goal.loc,
                                        point_cloud);

  navigation::DifferentialDomain::NullVisualizer viz;
  std::vector<navigation::DifferentialDomain::State> path;
  const bool found = navigation::AStar(start, goal, domain, &viz, &path);

  if (!found) {
    printf("No hybrid A* path found.\n");
    return;
  }

  const std::vector<Eigen::Vector3f> poses = DensifyPath(domain, path);

  std::vector<Eigen::Vector2f> path_points;
  path_points.reserve(poses.size());
  for (const auto& p : poses) path_points.emplace_back(p.x(), p.y());

  printf("Hybrid A* path found: %zu nodes, %zu dense poses, %.2f m/cell.\n",
         path.size(), poses.size(), params.grid.resolution);
  WritePath(FLAGS_path_output, poses);
  DrawScene(point_cloud, start.loc, goal.loc, path_points);
}

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, false);
    signal(SIGINT, SignalHandler);

    // Load Configurations
    const navigation::NavigationParams params =
        navigation::LoadConfig(FLAGS_nav_config);


    if (FLAGS_test_avoidance) {
        TestAvoidance(params);
    }
}