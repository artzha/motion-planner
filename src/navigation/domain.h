
// C headers.
#include <inttypes.h>

// C++ headers.
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

// Library headers.
#include "eigen3/Eigen/Dense"
#include "glog/logging.h"

// Project headers.
#include "navigation_params.h"
#include "shared/math/math_util.h"
#include "wavefront.h"

#ifndef DIFFERENTIAL_DOMAIN_H
#define DIFFERENTIAL_DOMAIN_H

namespace navigation {

// Kinodynamic hybrid-A* domain for a differential-drive (unicycle) robot.
//
// Nodes are continuous states (x, y, theta, v, omega). Each expansion samples a
// dynamic window of feasible (v, omega) controls (reachable from the current
// velocity within the acceleration limits over one primitive horizon),
// forward-simulates each control with the unicycle model, collision-checks the
// swept footprint against the base_link point cloud, and yields the continuous
// successor states. Only the pose (x, y, theta) is discretized into the search
// key; velocity rides along in the continuous state. The heuristic is an
// obstacle-aware wavefront cost-to-go.
struct DifferentialDomain {
  // ---- Tunable constants (motion/accel limits come from NavigationParams) ----
  // Heading buckets used by the pose key.
  static constexpr int kNumAngles = 16;
  // Discretization of the (v, omega) control grid.
  static constexpr int kNumV = 4;
  static constexpr int kNumW = 9;
  // Duration a constant control is held per edge (seconds). Larger values make
  // each edge cover more ground, so fewer expansions reach the goal.
  static constexpr float kPrimitiveDuration = 1.5f;
  // Integration/collision substep used during planning. Coarser than the
  // control-loop dt to speed up the search (fewer substeps per primitive);
  // must stay well below robot_radius / max_speed to avoid tunneling.
  static constexpr float kPlanningDt = 0.25f;
  // Position tolerance for reaching the goal (m); heading/velocity are free.
  static constexpr float kGoalTolerance = 0.2f;
  // Turn penalty (m per rad) so rotations are never zero-cost.
  static constexpr float kTurnPenalty = 0.1f;
  // Correlation length (m) of the cost-map noise field: larger = wider lanes.
  static constexpr float kNoiseCorrLength = 1.0f;

  struct State {
    Eigen::Vector2f loc = Eigen::Vector2f::Zero();  // x, y (m)
    float theta = 0.0f;                             // heading (rad)
    float v = 0.0f;                                 // linear speed (m/s), >= 0
    float omega = 0.0f;                             // angular rate (rad/s)

    State() = default;
    State(const Eigen::Vector2f& loc, float theta, float v, float omega)
        : loc(loc), theta(theta), v(v), omega(omega) {}
    State(float x, float y, float theta, float v, float omega)
        : loc(x, y), theta(theta), v(v), omega(omega) {}
  };

  // No-op visualizer (this project is ROS-free) that satisfies the AStar
  // Visualizer interface.
  struct NullVisualizer {
    void DrawEdge(const State&, const State&) {}
  };

  // seed / noise_eta parameterize the optional cost-map noise. noise_eta is the
  // magnitude (in meters) of a smooth potential added to edge costs; 0 (default)
  // disables it and reproduces the deterministic planner.
  DifferentialDomain(const navigation::NavigationParams& params,
                     const Eigen::Vector2f& origin,
                     float world_size,
                     const Eigen::Vector2f& goal,
                     const std::vector<Eigen::Vector2f>& point_cloud,
                     uint64_t seed = 0,
                     float noise_eta = 0.0f)
      : params_(params),
        origin_(origin),
        world_size_(world_size),
        res_(params.grid.resolution),
        kMapWidth(static_cast<int>(std::lround(world_size / params.grid.resolution))),
        kMapHeight(kMapWidth),
        dtheta_(2.0f * static_cast<float>(M_PI) / kNumAngles),
        robot_radius_(params.robot_width / 2.0f + params.obstacle_margin),
        point_cloud_(point_cloud),
        seed_(seed),
        noise_eta_(noise_eta) {
    // Precompute the (v, omega) control grid over the global velocity bounds.
    const float vmax = params_.linear_limits.max_speed;
    const float wmax = params_.angular_limits.max_speed;
    for (int i = 0; i < kNumV; ++i) {
      const float v = (kNumV == 1) ? 0.0f : vmax * i / (kNumV - 1);
      for (int j = 0; j < kNumW; ++j) {
        const float w = (kNumW == 1) ? 0.0f : -wmax + 2.0f * wmax * j / (kNumW - 1);
        controls_.emplace_back(v, w);
      }
    }

    // Obstacle-aware cost-to-go field, seeded at the goal.
    navigation::GridParams grid;
    grid.resolution = res_;
    grid.half_extent = world_size / 2.0f;
    wavefront_ = std::make_shared<motion_primitives::Wavefront>(
        grid, robot_radius_, goal, point_cloud);
  }

  // ---- Pose-only key (x, y, theta) ----

  uint64_t StateToKey(const State& s) const {
    int cx = static_cast<int>(std::lround((s.loc.x() - origin_.x()) / res_));
    int cy = static_cast<int>(std::lround((s.loc.y() - origin_.y()) / res_));
    math_util::Bound<int>(0, kMapWidth - 1, &cx);
    math_util::Bound<int>(0, kMapHeight - 1, &cy);
    const int ab = AngleBucket(s.theta);
    return (static_cast<uint64_t>(cy) * kMapWidth + cx) * kNumAngles + ab;
  }

  // Cell-center pose (v = omega = 0). Only used for optional debug/visualization;
  // path reconstruction uses the stored continuous states, not this.
  State KeyToState(uint64_t key) const {
    const int ab = key % kNumAngles;
    const uint64_t cell = key / kNumAngles;
    const int cx = cell % kMapWidth;
    const int cy = cell / kMapWidth;
    return State(cx * res_ + origin_.x(), cy * res_ + origin_.y(),
                 ab * dtheta_, 0.0f, 0.0f);
  }

  // ---- Heuristic and goal test ----

  float Heuristic(const State& s, const State& /*goal*/) const {
    return wavefront_->distance(s.loc);
  }

  bool AtGoal(const State& s, const State& goal) const {
    return (s.loc - goal.loc).norm() <= kGoalTolerance;
  }

  // ---- Successor generation (dynamic window + forward simulation) ----

  void GetSuccessors(const State& s,
                     std::vector<State>* succ,
                     std::vector<float>* costs) const {
    succ->clear();
    costs->clear();
    const float T = kPrimitiveDuration;
    const float vmax = params_.linear_limits.max_speed;
    const float wmax = params_.angular_limits.max_speed;

    // Dynamic window: velocities reachable within the accel limits over T.
    const float v_lo =
        std::max(0.0f, s.v - params_.linear_limits.max_deceleration * T);
    const float v_hi =
        std::min(vmax, s.v + params_.linear_limits.max_acceleration * T);
    const float w_acc = params_.angular_limits.max_acceleration;
    const float w_lo = std::max(-wmax, s.omega - w_acc * T);
    const float w_hi = std::min(wmax, s.omega + w_acc * T);

    for (const auto& c : controls_) {
      const float v_c = c.first;
      const float w_c = c.second;
      if (v_c < v_lo - 1e-6f || v_c > v_hi + 1e-6f) continue;
      if (w_c < w_lo - 1e-6f || w_c > w_hi + 1e-6f) continue;
      if (v_c == 0.0f && w_c == 0.0f) continue;  // no-op

      bool collision = false;
      const std::vector<State> roll = Rollout(s, v_c, w_c, &collision);
      if (collision || roll.empty()) continue;
      const State& end = roll.back();
      if (!InBounds(end.loc)) continue;

      succ->push_back(end);
      // Base arc-length + turn cost, plus a smooth non-negative cost-map
      // potential (eta * N in [0, eta]) so edges stay strictly positive.
      const float base = std::fabs(v_c) * T + kTurnPenalty * std::fabs(w_c) * T;
      costs->push_back(base + noise_eta_ * SmoothNoise(end.loc));
    }
  }

  // Integrates a constant control (v_c, w_c) from `from` over the primitive
  // horizon, substepping at params_.dt. Returns the swept poses (endpoint last),
  // and sets *collision if the footprint disk hits the cloud. Shared by the
  // successor generator and the path densifier.
  std::vector<State> Rollout(const State& from, float v_c, float w_c,
                             bool* collision) const {
    std::vector<State> states;
    const float dt = kPlanningDt;
    const int n =
        std::max(1, static_cast<int>(std::lround(kPrimitiveDuration / dt)));
    float x = from.loc.x();
    float y = from.loc.y();
    float th = from.theta;
    bool hit = false;
    for (int i = 0; i < n; ++i) {
      th += w_c * dt;
      x += v_c * std::cos(th) * dt;
      y += v_c * std::sin(th) * dt;
      const Eigen::Vector2f p(x, y);
      if (PointBlocked(p)) {
        hit = true;
        break;
      }
      states.emplace_back(p, th, v_c, w_c);
    }
    if (collision) *collision = hit;
    return states;
  }

  // ---- Collision / bounds ----

  bool PointBlocked(const Eigen::Vector2f& p) const {
    const float r2 = robot_radius_ * robot_radius_;
    for (const auto& q : point_cloud_) {
      if ((p - q).squaredNorm() < r2) return true;
    }
    return false;
  }

  bool InBounds(const Eigen::Vector2f& p) const {
    return p.x() >= origin_.x() && p.x() <= origin_.x() + world_size_ &&
           p.y() >= origin_.y() && p.y() <= origin_.y() + world_size_;
  }

 private:
  int AngleBucket(float theta) const {
    float a = std::fmod(theta, 2.0f * static_cast<float>(M_PI));
    if (a < 0.0f) a += 2.0f * static_cast<float>(M_PI);
    int ab = static_cast<int>(std::lround(a / dtheta_)) % kNumAngles;
    if (ab < 0) ab += kNumAngles;
    return ab;
  }

  // ---- Smooth cost-map noise (deterministic value noise) ----

  static float Fade(float t) { return t * t * t * (t * (t * 6 - 15) + 10); }

  // Hash of integer lattice node (ix, iy) and the seed to a value in [0, 1).
  float LatticeValue(int ix, int iy) const {
    uint64_t h = (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) ^
                 static_cast<uint32_t>(iy) ^ (seed_ + 0x9e3779b97f4a7c15ULL);
    h = (h ^ (h >> 30)) * 0xbf58476d1ce4e5b9ULL;  // splitmix64
    h = (h ^ (h >> 27)) * 0x94d049bb133111ebULL;
    h ^= (h >> 31);
    return (h >> 11) * (1.0f / 9007199254740992.0f);
  }

  // Low-frequency value noise in [0, 1): random values on a lattice with spacing
  // kNoiseCorrLength, smootherstep-interpolated. Unlike per-cell white noise this
  // is spatially correlated, so it shifts which lane looks cheaper instead of
  // just adding jitter that discretizes away.
  float SmoothNoise(const Eigen::Vector2f& p) const {
    const float s = p.x() / kNoiseCorrLength;
    const float t = p.y() / kNoiseCorrLength;
    const int x0 = static_cast<int>(std::floor(s));
    const int y0 = static_cast<int>(std::floor(t));
    const float fx = Fade(s - x0);
    const float fy = Fade(t - y0);
    const float a = LatticeValue(x0, y0) +
                    fx * (LatticeValue(x0 + 1, y0) - LatticeValue(x0, y0));
    const float b = LatticeValue(x0, y0 + 1) +
                    fx * (LatticeValue(x0 + 1, y0 + 1) - LatticeValue(x0, y0 + 1));
    return a + fy * (b - a);
  }

  navigation::NavigationParams params_;
  Eigen::Vector2f origin_;
  float world_size_;
  float res_;
  int kMapWidth;
  int kMapHeight;
  float dtheta_;
  float robot_radius_;
  std::vector<Eigen::Vector2f> point_cloud_;
  uint64_t seed_;
  float noise_eta_;
  std::vector<std::pair<float, float>> controls_;
  std::shared_ptr<motion_primitives::Wavefront> wavefront_;
};

}  // namespace navigation
#endif  // DIFFERENTIAL_DOMAIN_H
