#ifndef WAVEFRONT_H
#define WAVEFRONT_H

#include <functional>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "navigation_params.h"

namespace motion_primitives {

// Obstacle-aware cost-to-go field over a local occupancy grid in base_link
// (+x forward, +y left, origin at the grid center). Cloud points inflated by
// the robot radius mark occupied cells; a wavefront seeded at the goal fills
// every reachable free cell with its cost-to-go.
class Wavefront {
 public:
  // Extra cost per meter travelled, sampled at a cell center, on top of the
  // distance itself: a field of 0 leaves plain distance-to-go. Supplying one is
  // what keeps this usable as an A* heuristic for a search whose edges carry
  // that same term -- an estimate blind to it underestimates the cost-to-go by
  // however much the remaining route must pay, and the search degenerates
  // toward Dijkstra exactly where the term is largest.
  using ExtraCostPerMeter = std::function<float(const Eigen::Vector2f&)>;

  // With no extra-cost field the fill is a uniform-cost BFS; with one it is a
  // Dijkstra, which costs a priority queue over the same cells.
  Wavefront(const navigation::GridParams& grid,
            const float inflation_radius,
            const Eigen::Vector2f& goal,
            const std::vector<Eigen::Vector2f>& point_cloud,
            const ExtraCostPerMeter& extra_cost_per_meter = {});

  float distance(const Eigen::Vector2f& point) const;

  float idealHeading(const Eigen::Vector2f& point) const;

  static constexpr float kUnreachable = 1e6f;

 private:
  int index(const int ix, const int iy) const { return iy * width_ + ix; }
  bool toCell(const Eigen::Vector2f& point, int& ix, int& iy) const;
  Eigen::Vector2f cellCenter(const int ix, const int iy) const;
  void fillUniform(const int gx, const int gy);
  void fillWeighted(const int gx, const int gy,
                    const ExtraCostPerMeter& extra_cost_per_meter);

  navigation::GridParams grid_;
  Eigen::Vector2f goal_;
  int center_;
  int width_;
  std::vector<float> cost_;
  std::vector<bool> occupied_;
};

}  // namespace motion_primitives

#endif  // WAVEFRONT_H
