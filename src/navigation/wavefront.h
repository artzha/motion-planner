#ifndef WAVEFRONT_H
#define WAVEFRONT_H

#include <vector>

#include "eigen3/Eigen/Dense"
#include "navigation_params.h"

namespace motion_primitives {

// Obstacle-aware cost-to-go field over a local occupancy grid in base_link
// (+x forward, +y left, origin at the grid center). Cloud points inflated by
// the robot radius mark occupied cells; a BFS wavefront seeded at the goal
// fills every reachable free cell with its distance-to-goal.
class Wavefront {
 public:
  Wavefront(const navigation::GridParams& grid,
            const float inflation_radius,
            const Eigen::Vector2f& goal,
            const std::vector<Eigen::Vector2f>& point_cloud);

  float distance(const Eigen::Vector2f& point) const;

  float idealHeading(const Eigen::Vector2f& point) const;

  static constexpr float kUnreachable = 1e6f;

 private:
  int index(const int ix, const int iy) const { return iy * width_ + ix; }
  bool toCell(const Eigen::Vector2f& point, int& ix, int& iy) const;

  navigation::GridParams grid_;
  Eigen::Vector2f goal_;
  int center_;
  int width_;
  std::vector<float> cost_;
  std::vector<bool> occupied_;
};

}  // namespace motion_primitives

#endif  // WAVEFRONT_H
