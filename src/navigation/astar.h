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
//   float    HeuristicBase(const State&, const State& goal)
//   bool     AtGoal(const State&, const State& goal)
//   void     GetSuccessors(const State&, vector<State>*, vector<float>* costs,
//                          vector<float>* base_costs)
// heuristic_weight w >= 1 gives weighted A* (f = g + w*h): larger w is greedier
// and faster but sacrifices optimality. w = 1 (default) is standard A*.
//
// A Domain may charge more than the true edge cost -- to repel a search away
// from paths already taken, say -- and reports that inflated value in `costs`
// and the true one in `base_costs`. It estimates each with its own heuristic:
// Heuristic() for what is being minimized, HeuristicBase() for cost alone. The
// two are tracked separately so both of the following work on the honest cost:
//   cost_bound  when > 0, prunes any node that cannot be completed within this
//               base cost, giving bounded suboptimality. Tested with
//               HeuristicBase, which underestimates the base cost, so a node
//               already over budget by g_base + h_base can never come back under
//               it. Testing it with the inflated estimate instead would discard
//               nodes over budget only in repulsion -- that is, precisely the
//               detours the bound is there to permit.
//   out_cost    when non-null, receives the base cost of the returned path.
template <class Domain, class Visualizer>
bool AStar(const typename Domain::State& start,
           const typename Domain::State& goal,
           const Domain& domain,
           Visualizer* const viz,
           std::vector<typename Domain::State>* path,
           float heuristic_weight = 1.0f,
           float cost_bound = 0.0f,
           float* out_cost = nullptr) {
  using State = typename Domain::State;
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  // Safety cap on expansions. This bounds worst-case (no-solution) runtime: the
  // search fans out over free space until it hits this cap, so the ceiling is
  // roughly kMaxEdgeExpansions * per-expansion cost. Sized to keep the worst
  // case under ~0.2 s while still solving every goal in the demo scene.
  static const uint64_t kMaxEdgeExpansions = 7000;
  static const bool kDebug = false;

  std::unordered_map<uint64_t, uint64_t> parent_map_;
  // G-values of discovered nodes.
  std::unordered_map<uint64_t, float> g_values_;
  // The same paths measured without the Domain's diversity term. Carried for
  // whichever path currently has the best g, since that is the one that would
  // be returned, so this describes the returned path's real cost.
  std::unordered_map<uint64_t, float> g_base_values_;
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
  g_base_values_[k_start] = 0;
  state_map_[k_start] = start;
  queue.push(
      {AStarPriority(0, heuristic_weight * domain.Heuristic(start, goal)),
       k_start});

  const double t_start = GetMonotonicTime();

  // Re-used across expansions to limit memory re-allocations.
  std::vector<State> succ;
  std::vector<float> costs;
  std::vector<float> base_costs;
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
      if (out_cost != nullptr) *out_cost = g_base_values_[k_current];
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
    domain.GetSuccessors(s_current, &succ, &costs, &base_costs);
    for (size_t i = 0; i < succ.size(); ++i) {
      const State& s_next = succ[i];
      const uint64_t k_next = domain.StateToKey(s_next);
      if (closed_set_.find(k_next) != closed_set_.end()) continue;
      const float g = g_values_[k_current] + costs[i];
      const float g_base = g_base_values_[k_current] + base_costs[i];
      // Hoisted above the relaxation check so the bound can use it. Both are
      // O(1) grid lookups, so evaluating them for successors that turn out not
      // to improve on a known g is free.
      const float h = domain.Heuristic(s_next, goal);
      if (cost_bound > 0.0f &&
          g_base + domain.HeuristicBase(s_next, goal) > cost_bound) {
        continue;
      }
      const auto it = g_values_.find(k_next);
      if (it == g_values_.end() || g < it->second) {
        parent_map_[k_next] = k_current;
        g_values_[k_next] = g;
        g_base_values_[k_next] = g_base;
        state_map_[k_next] = s_next;
        queue.push({AStarPriority(g, heuristic_weight * h), k_next});
        viz->DrawEdge(s_current, s_next);
      }
    }
  }
  const double t_end = GetMonotonicTime();
  // Not necessarily an error: a caller searching under a cost_bound uses
  // failure as its stop condition, so this is expected traffic there.
  if (kDebug) printf("No path found, took %f seconds.\n", t_end - t_start);
  // Priority queue is exhausted (or the bound pruned everything), so no path
  // exists within the budget.
  return false;
}


}  // namespace navigation
#endif  // A_STAR_H