#ifndef NAVIGATION_CONFIG_H
#define NAVIGATION_CONFIG_H

#include <string>

#include "navigation_params.h"

namespace navigation {

// Loads NavigationParams (and the nested GridParams) from a navigation lua
// config file via the AMRL config_reader.
NavigationParams LoadConfig(const std::string& lua_path);

}  // namespace navigation

#endif  // NAVIGATION_CONFIG_H
