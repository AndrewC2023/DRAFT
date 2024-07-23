add_library(
    ThreeD

    GridManager.cpp
    Planners/AStarGrid.cpp
)

target_link_libraries(
    ThreeD
    
    Config
    ${EIGEN3_LIBRARIES}
)

add_subdirectory(Planners)
add_subdirectory(Validators)
add_subdirectory(Tests)