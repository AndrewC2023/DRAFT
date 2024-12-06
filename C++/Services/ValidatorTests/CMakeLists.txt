add_executable(
    RaytracingValidatorTest

    TestRayTracing.cpp
)

target_link_libraries(
    RaytracingValidatorTest

    TwoD
    BackendVisualization
    Config
)