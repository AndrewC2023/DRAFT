add_subdirectory(Planners)
add_subdirectory(Validators)
add_subdirectory(Tests)

add_library(
    TwoD
        2DGridManager.cpp

)

target_link_libraries(
    TwoD
    
    Config
    ${EIGEN3_LIBRARIES}
)