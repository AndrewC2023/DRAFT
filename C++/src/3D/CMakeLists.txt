add_library(
    ThreeD

    GridManager.cpp
    # Planners/AStar/AstarGrid.cpp
)

target_link_libraries(
    ThreeD
    
    Config
    ${EIGEN3_LIBRARIES}
    pthread
)

add_subdirectory(Planners)
add_subdirectory(Validators)
add_subdirectory(Tests)