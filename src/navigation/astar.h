// C headers.
#include <inttypes.h>

// C++ headers.
#include <algorithm>
#include <cstdio>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Library headers.
#include "eigen3/Eigen/Dense"
#include "glog/logging.h"

// Project headers.
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#ifndef A_STAR_H
#define A_STAR_H

namespace navigation {

// Struct to keep track of node priorities in A*, with tie-breaking to
// prefer expansion of nodes farther along the path.
struct AStarPriority {
  AStarPriority() {}

  AStarPriority(float g, float h) : g(g), h(h) {}

  // Returns true iff this has lower priority than the other.
  // Note that higher cost => lower priority.
  bool operator<(const AStarPriority& other) const {
    const float f = g + h;
    const float f_other = other.g + other.h;
    // This has lower priority if its total cost is higher.
    if (f > f_other + kEpsilon) return true;
    if (f < f_other - kEpsilon) return false;
    // Tie-breaking when costs are the same:
    // This has lower priority if its g-value is lower.
    return (g < other.g);
  }

  // Returns true iff this has higher priority than the other.
  // Note that lower cost => higher priority.
  bool operator>(const AStarPriority& other) const {
    const float f = g + h;
    const float f_other = other.g + other.h;
    // This has higher priority if its total cost is lower.
    if (f < f_other - kEpsilon) return true;
    if (f > f_other + kEpsilon) return false;
    // Tie-breaking when costs are the same:
    // This has higher priority if its g-value is higher.
    return (g > other.g);
  }

  // Epsilon for float comparisons.
  static constexpr float kEpsilon = 1e-3;
  // Cost to go: cost from start to this node.
  float g;
  // Heuristic: estimated cost from this node to the goal.
  float h;
};

// Open-list entry for the binary heap (std::priority_queue).
struct QueueEntry {
  AStarPriority priority;
  uint64_t key;
};

// Orders the heap so that the highest-priority (lowest f = g + h) entry is on
// top: std::priority_queue pops the element that is not `<` any other, and
// AStarPriority::operator< means "lower priority".
struct QueueCompare {
  bool operator()(const QueueEntry& a, const QueueEntry& b) const {
    return a.priority < b.priority;
  }
};

// Kinodynamic hybrid A*. Nodes are continuous Domain::States; the search key
// (Domain::StateToKey) discretizes them for the open/closed sets while the true
// continuous state is stored per key for expansion and path reconstruction.
// The Domain must provide:
//   uint64_t StateToKey(const State&)
//   float    Heuristic(const State&, const State& goal)
//   bool     AtGoal(const State&, const State& goal)
//   void     GetSuccessors(const State&, vector<State>*, vector<float>* costs)
template <class Domain, class Visualizer>
bool AStar(const typename Domain::State& start,
           const typename Domain::State& goal,
           const Domain& domain,
           Visualizer* const viz,
           std::vector<typename Domain::State>* path) {
  using State = typename Domain::State;
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  // Safety cap on expansions. This bounds worst-case (no-solution) runtime: the
  // search fans out over free space until it hits this cap, so the ceiling is
  // roughly kMaxEdgeExpansions * per-expansion cost. Sized to keep the worst
  // case under ~0.2 s while still solving every goal in the demo scene.
  static const uint64_t kMaxEdgeExpansions = 8500;
  static const bool kDebug = false;

  std::unordered_map<uint64_t, uint64_t> parent_map_;
  // G-values of discovered nodes.
  std::unordered_map<uint64_t, float> g_values_;
  // Best continuous state reaching each key (for expansion + reconstruction).
  std::unordered_map<uint64_t, State> state_map_;
  // Closed set (keys with finalized costs).
  std::unordered_set<uint64_t> closed_set_;
  // Open list: a binary min-heap on f = g + h. Priorities are never updated in
  // place; a better g pushes a new entry and stale entries are discarded on pop
  // via the closed-set check (lazy deletion).
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> queue;

  const uint64_t k_start = domain.StateToKey(start);
  g_values_[k_start] = 0;
  state_map_[k_start] = start;
  queue.push({AStarPriority(0, domain.Heuristic(start, goal)), k_start});

  const double t_start = GetMonotonicTime();

  // Re-used across expansions to limit memory re-allocations.
  std::vector<State> succ;
  std::vector<float> costs;
  uint64_t edge_expansions = 0;
  while (edge_expansions < kMaxEdgeExpansions && !queue.empty()) {
    const uint64_t k_current = queue.top().key;
    queue.pop();
    // Lazy deletion: skip keys already finalized (or superseded by a lower-cost
    // entry that was popped earlier).
    if (closed_set_.find(k_current) != closed_set_.end()) continue;
    ++edge_expansions;
    const State s_current = state_map_[k_current];

    // Continuous goal test on expansion (goal is a region, not a single key).
    if (domain.AtGoal(s_current, goal)) {
      const double t_end = GetMonotonicTime();
      if (kDebug) printf("Path found in %f seconds.\n", t_end - t_start);
      path->clear();
      uint64_t current = k_current;
      while (true) {
        path->push_back(state_map_[current]);
        if (current == k_start) break;
        current = parent_map_[current];
      }
      std::reverse(path->begin(), path->end());
      return true;
    }

    closed_set_.insert(k_current);
    domain.GetSuccessors(s_current, &succ, &costs);
    for (size_t i = 0; i < succ.size(); ++i) {
      const State& s_next = succ[i];
      const uint64_t k_next = domain.StateToKey(s_next);
      if (closed_set_.find(k_next) != closed_set_.end()) continue;
      const float g = g_values_[k_current] + costs[i];
      const auto it = g_values_.find(k_next);
      if (it == g_values_.end() || g < it->second) {
        parent_map_[k_next] = k_current;
        g_values_[k_next] = g;
        state_map_[k_next] = s_next;
        const float h = domain.Heuristic(s_next, goal);
        queue.push({AStarPriority(g, h), k_next});
        viz->DrawEdge(s_current, s_next);
      }
    }
  }
  const double t_end = GetMonotonicTime();
  printf("No path found, took %f seconds.\n", t_end - t_start);
  // Priority queue is exhausted, but path not found. No path exists.
  return false;
}


}  // namespace navigation
#endif  // A_STAR_H