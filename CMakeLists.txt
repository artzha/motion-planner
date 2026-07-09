cmake_minimum_required(VERSION 3.16)

project(motion_planner
  VERSION 0.1.0
  DESCRIPTION "Motion planner"
  LANGUAGES CXX)

message(STATUS "Compiling motion_planner")
message(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

# ---------------------------------------------------------------------------
# Dependencies (ROS-free)
#   - Eigen3 / Lua headers are found on the default include path (/usr/include).
#   - glog pulls in gflags; both are linked by name, matching the AMRL layout.
# ---------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(Lua 5.2 REQUIRED)

# Build all objects position-independent so the static amrl_shared_lib can be
# linked into the Python extension module (a shared object).
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# The AMRL shared library builds itself as `amrl_shared_lib` (glog + pthread).
add_subdirectory(src/shared)

# ---------------------------------------------------------------------------
# Navigation executable (1D TOC + Ackermann sampler + obstacle checker)
# ---------------------------------------------------------------------------
add_executable(navigation
  src/navigation/navigation_main.cc
  src/navigation/navigation_config.cc
  src/navigation/motion_primitives.cc
  src/navigation/constant_curvature_arc.cc
  src/navigation/ackermann_motion_primitives.cc
  src/navigation/differential_motion_primitives.cc
  src/navigation/wavefront.cc)

target_include_directories(navigation PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/navigation)

# `#include "shared/math/..."` resolves from src/; submodule + third-party
# headers are marked SYSTEM so their warnings don't leak into this build.
target_include_directories(navigation SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/config_reader/include
  ${LUA_INCLUDE_DIR})

target_link_libraries(navigation PRIVATE
  amrl_shared_lib
  Eigen3::Eigen
  ${LUA_LIBRARIES}
  glog
  gflags
  pthread)

# ---------------------------------------------------------------------------
# Optional Python bindings (pybind11). Built only when pybind11 is discoverable
# via the active Python interpreter (e.g. `pip install pybind11`).
# ---------------------------------------------------------------------------
find_package(Python3 COMPONENTS Interpreter Development)
if(Python3_FOUND)
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
    OUTPUT_VARIABLE _pybind11_cmakedir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _pybind11_query_rc)
  if(_pybind11_query_rc EQUAL 0)
    list(APPEND CMAKE_PREFIX_PATH "${_pybind11_cmakedir}")
    find_package(pybind11 CONFIG QUIET)
  endif()
endif()

if(pybind11_FOUND)
  message(STATUS "Building Python bindings: hybrid_astar")
  pybind11_add_module(hybrid_astar
    src/navigation/python_bindings.cc
    src/navigation/navigation_config.cc
    src/navigation/wavefront.cc)

  target_include_directories(hybrid_astar PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/navigation)
  target_include_directories(hybrid_astar SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config_reader/include
    ${LUA_INCLUDE_DIR})
  target_link_libraries(hybrid_astar PRIVATE
    amrl_shared_lib
    Eigen3::Eigen
    ${LUA_LIBRARIES}
    glog
    gflags
    pthread)
else()
  message(STATUS "pybind11 not found; skipping Python bindings (hybrid_astar)")
endif()
