cmake_minimum_required (VERSION 3.11)

project(AlgorithmDevelopment)

set( CMAKE_CXX_STANDARD 20 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

# Add libraries
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/src" )

# YAML library
find_package(YAMLCPP REQUIRED REQUIRED)
include_directories(${YAMLCPP_INCLUDE_DIRS})

# Eigen Library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

### Targets
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/services")