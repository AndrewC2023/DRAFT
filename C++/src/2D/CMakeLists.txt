add_subdirectory(Planners)
add_subdirectory(Validators)

add_library(
    TwoD
        2DGridManager.cpp
        2DHybridGridManager.cpp
        Planners/RRTStar/RRTStar2D.cpp
        Validators/RayTracingValidator.cpp
        Validators/RayTracingValidatorUncertain.cpp
        
)

target_link_libraries(
    TwoD
    
    Config
    ${EIGEN3_LIBRARIES}
    BackendVisualization
)