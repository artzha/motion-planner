#include "motion_planner.h"

#include "config_reader/config_reader.h"
#include "util/timer.h"

namespace motion_planner {

std::string Version() { return MOTION_PLANNER_VERSION; }

double MonotonicTime() { return GetMonotonicTime(); }

}  // namespace motion_planner
