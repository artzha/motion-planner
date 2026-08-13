#include "wavefront.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <utility>
#include <vector>

namespace motion_primitives {

namespace {
constexpr int kNeighborDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr int kNeighborDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
}  // namespace

Wavefront::Wavefront(const navigation::GridParams& grid,
                     const float inflation_radius,
                     const Eigen::Vector2f& goal,
                     const std::vector<Eigen::Vector2f>& point_cloud,
                     const ExtraCostPerMeter& extra_cost_per_meter)
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

  if (extra_cost_per_meter) {
    fillWeighted(gx, gy, extra_cost_per_meter);
  } else {
    fillUniform(gx, gy);
  }
}

// Every step charges one cell width, diagonals included. That understates a
// diagonal by up to a factor of sqrt(2), which is the safe direction to err in:
// the result is a lower bound on the distance, so it stays admissible as a
// heuristic.
void Wavefront::fillUniform(const int gx, const int gy) {
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

// As above, but a step's cost rises with the extra cost of the cell entered, so
// cells are no longer settled in the order they are reached and the queue has to
// be a priority queue. The extra cost is charged over the step length, which is
// what makes it commensurate with the distance it is added to.
void Wavefront::fillWeighted(const int gx, const int gy,
                             const ExtraCostPerMeter& extra_cost_per_meter) {
  using Entry = std::pair<float, int>;  // cost, cell index
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier;
  cost_[index(gx, gy)] = 0.0f;
  frontier.push({0.0f, index(gx, gy)});
  while (!frontier.empty()) {
    const auto [c, ci] = frontier.top();
    frontier.pop();
    if (c > cost_[ci]) continue;  // stale entry, already settled cheaper
    const int cx = ci % width_;
    const int cy = ci / width_;
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + kNeighborDx[k];
      const int ny = cy + kNeighborDy[k];
      if (nx < 0 || nx >= width_ || ny < 0 || ny >= width_) continue;
      const int ni = index(nx, ny);
      if (occupied_[ni]) continue;
      const float extra =
          std::max(0.0f, extra_cost_per_meter(cellCenter(nx, ny)));
      const float step = grid_.resolution * (1.0f + extra);
      if (c + step >= cost_[ni]) continue;
      cost_[ni] = c + step;
      frontier.push({cost_[ni], ni});
    }
  }
}

Eigen::Vector2f Wavefront::cellCenter(const int ix, const int iy) const {
  return Eigen::Vector2f(static_cast<float>(ix - center_) * grid_.resolution,
                         static_cast<float>(iy - center_) * grid_.resolution);
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
