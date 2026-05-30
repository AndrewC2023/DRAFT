add_library(
    BackendVisualization

    2DVisualizer.cpp
)

target_link_libraries(
    BackendVisualization

    TwoD
    Config
    ${MATPLOT_LIBRARIES}
)