cmake_minimum_required (VERSION 3.11)

project(AlgorithmDevelopment)

set( CMAKE_CXX_STANDARD 20 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/admin/cmake_modules/")

# Add libraries
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/src" )

# YAML library
find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})

# Eigen Library
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Matplot++
find_package(Matplot++ REQUIRED)

## Configures cmake to output binaries in build/bin
## make sure to run cmake configuration command cmake -S . -B build in the C++ directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

### Targets
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/Services")

