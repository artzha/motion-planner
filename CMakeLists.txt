cmake_minimum_required(VERSION 3.16)

project(motion_planner
  VERSION 0.1.0
  DESCRIPTION "Motion planner"
  LANGUAGES CXX)

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ---------------------------------------------------------------------------
# Dependencies. Eigen3/Lua/glog/gflags come from the system. ROS 2 (ament) is
# optional: when it is available (colcon), config_reader and amrl_shared_lib are
# resolved as installed ament packages. Otherwise (plain cmake / uv venv) they
# are built from the vendored submodules under src/, so ROS is not required.
# ---------------------------------------------------------------------------
find_package(ament_cmake QUIET)
set(_use_ament ${ament_cmake_FOUND})

# Position-independent so the static-ish lib (and the vendored amrl_shared_lib
# built below) can be linked into the Python extension module (a shared object).
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 REQUIRED)
find_package(Lua 5.2 REQUIRED)
find_library(GLOG_LIBRARY NAMES glog REQUIRED)
find_library(GFLAGS_LIBRARY NAMES gflags REQUIRED)

if(_use_ament)
  find_package(config_reader REQUIRED)
  find_package(amrl_shared_lib REQUIRED)
  set(_planner_dep_libs config_reader::config_reader amrl_shared_lib::amrl_shared_lib)
else()
  # amrl_shared_lib is a plain-CMake static lib; config_reader is header-only
  # (its own CMakeLists needs ament, so wire its headers via include dirs below).
  add_subdirectory(src/shared)
  set(_planner_dep_libs amrl_shared_lib)
endif()

# ---------------------------------------------------------------------------
# Core planning library (reusable primitives / sampler / evaluator / config).
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME}
  src/navigation/navigation_config.cc
  src/navigation/motion_primitives.cc
  src/navigation/constant_curvature_arc.cc
  src/navigation/ackermann_motion_primitives.cc
  src/navigation/differential_motion_primitives.cc
  src/navigation/hybrid_planner.cc
  src/navigation/wavefront.cc)

# Own headers are included unprefixed at build time (src/navigation on the path).
# Consumers include them as "motion_planner/<header>.h"; the headers' internal
# unprefixed cross-includes then resolve via the exported include/motion_planner.
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/navigation>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/motion_planner>)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${LUA_INCLUDE_DIR})

if(NOT _use_ament)
  # Sources include amrl headers both as "shared/..." and bare "math/.."/"util/.."
  # and config_reader as "config_reader/config_reader.h"; in the non-ament build
  # the vendored roots must be on the include path.
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/shared>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/config_reader/include>)
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${_planner_dep_libs}
  Eigen3::Eigen
  ${LUA_LIBRARIES}
  ${GLOG_LIBRARY}
  ${GFLAGS_LIBRARY}
  pthread)

# ---------------------------------------------------------------------------
# navigation executable (hybrid A* demo / CLI).
# ---------------------------------------------------------------------------
add_executable(navigation src/navigation/navigation_main.cc)
target_link_libraries(navigation PRIVATE ${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Python bindings (pybind11). Unchanged from upstream: exposes only the
# hybrid A* `hybrid_astar.plan()` used by visor. Built against the ROS-distro
# Python (3.12 on Kilted, matching the visor uv venv).
# ---------------------------------------------------------------------------
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG QUIET)
if(NOT pybind11_FOUND)
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
    OUTPUT_VARIABLE _pybind11_cmakedir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _pybind11_query_rc)
  if(_pybind11_query_rc EQUAL 0)
    list(APPEND CMAKE_PREFIX_PATH "${_pybind11_cmakedir}")
    find_package(pybind11 CONFIG REQUIRED)
  endif()
endif()

if(pybind11_FOUND)
  message(STATUS "Building Python bindings module: hybrid_astar")
  pybind11_add_module(hybrid_astar src/navigation/python_bindings.cc)
  target_link_libraries(hybrid_astar PRIVATE ${PROJECT_NAME})

  # Install onto the ROS PYTHONPATH so `import hybrid_astar` works once the
  # workspace overlay is sourced.
  set(_py_site
    "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
  install(TARGETS hybrid_astar
    LIBRARY DESTINATION "${_py_site}")
else()
  message(WARNING "pybind11 not found; skipping hybrid_astar Python module")
endif()

# ---------------------------------------------------------------------------
# Install: library + exported CMake config, headers, executable, config.
# ---------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(TARGETS navigation
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY src/navigation/
  DESTINATION include/motion_planner
  FILES_MATCHING PATTERN "*.h")

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(_use_ament)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_include_directories(include include/motion_planner)
  ament_export_dependencies(config_reader amrl_shared_lib Eigen3)

  ament_package()
endif()
