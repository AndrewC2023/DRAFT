cmake_minimum_required (VERSION 3.20)

project(
    Draft
    VERSION 0.1
    DESCRIPTION "Dynamics, Routing, Autonomy, and Feasibility Testbed. A testbed for dynamical simulations with perception and navigation capabilities for the development of planning and autonomy systems. Full autonomy stacks can be built and simulated in this testbed."
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# No options yet, we will have lots of different ways to build the project so we will focus on build options later.

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(GTest REQUIRED)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_subdirectory(src)
add_subdirectory(Services)