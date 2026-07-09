#include "wavefront.h"

#include <cmath>
#include <queue>

namespace motion_primitives {

namespace {
constexpr int kNeighborDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr int kNeighborDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
}  // namespace

Wavefront::Wavefront(const navigation::GridParams& grid,
                     const float inflation_radius,
                     const Eigen::Vector2f& goal,
                     const std::vector<Eigen::Vector2f>& point_cloud)
    : grid_(grid), goal_(goal) {
  center_ = static_cast<int>(std::lround(grid_.half_extent / grid_.resolution));
  width_ = 2 * center_ + 1;
  cost_.assign(width_ * width_, kUnreachable);
  occupied_.assign(width_ * width_, false);

  const int inflation_cells =
      static_cast<int>(std::ceil(inflation_radius / grid_.resolution));
  for (const auto& point : point_cloud) {
    int ix, iy;
    if (!toCell(point, ix, iy)) continue;
    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        const int nx = ix + dx;
        const int ny = iy + dy;
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= width_) continue;
        occupied_[index(nx, ny)] = true;
      }
    }
  }

  int gx, gy;
  if (!toCell(goal_, gx, gy)) return;  // goal outside grid: leave all unreachable
  occupied_[index(gx, gy)] = false;    // never block the seed cell

  std::queue<std::pair<int, int>> frontier;
  cost_[index(gx, gy)] = 0.0f;
  frontier.push({gx, gy});
  while (!frontier.empty()) {
    const auto [cx, cy] = frontier.front();
    frontier.pop();
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + kNeighborDx[k];
      const int ny = cy + kNeighborDy[k];
      if (nx < 0 || nx >= width_ || ny < 0 || ny >= width_) continue;
      const int ni = index(nx, ny);
      if (occupied_[ni] || cost_[ni] != kUnreachable) continue;
      cost_[ni] = cost_[index(cx, cy)] + grid_.resolution;
      frontier.push({nx, ny});
    }
  }
}

bool Wavefront::toCell(const Eigen::Vector2f& point, int& ix, int& iy) const {
  ix = static_cast<int>(std::lround(point.x() / grid_.resolution)) + center_;
  iy = static_cast<int>(std::lround(point.y() / grid_.resolution)) + center_;
  return ix >= 0 && ix < width_ && iy >= 0 && iy < width_;
}

float Wavefront::distance(const Eigen::Vector2f& point) const {
  int ix, iy;
  if (!toCell(point, ix, iy)) return kUnreachable;
  return cost_[index(ix, iy)];
}

float Wavefront::idealHeading(const Eigen::Vector2f& point) const {
  int ix, iy;
  if (!toCell(point, ix, iy)) return std::atan2(goal_.y(), goal_.x());

  int best_dx = 0, best_dy = 0;
  float best_cost = cost_[index(ix, iy)];
  for (int k = 0; k < 8; ++k) {
    const int nx = ix + kNeighborDx[k];
    const int ny = iy + kNeighborDy[k];
    if (nx < 0 || nx >= width_ || ny < 0 || ny >= width_) continue;
    const float c = cost_[index(nx, ny)];
    if (c < best_cost) {
      best_cost = c;
      best_dx = kNeighborDx[k];
      best_dy = kNeighborDy[k];
    }
  }

  if (best_dx == 0 && best_dy == 0) {
    const Eigen::Vector2f to_goal = goal_ - point;
    return std::atan2(to_goal.y(), to_goal.x());
  }
  return std::atan2(static_cast<float>(best_dy), static_cast<float>(best_dx));
}

}  // namespace motion_primitives
