#ifndef MOTION_PLANNER_MOTION_PLANNER_H_
#define MOTION_PLANNER_MOTION_PLANNER_H_

#include <string>

namespace motion_planner {

// Returns the semantic version string of the library (e.g. "0.1.0").
std::string Version();

// High-resolution monotonic clock time in seconds (via the AMRL shared lib).
double MonotonicTime();

}  // namespace motion_planner

#endif  // MOTION_PLANNER_MOTION_PLANNER_H_
