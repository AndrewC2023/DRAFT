add_library(
    backendVisualization

    2DVisualizer.cpp
)

target_link_libraries(
    backendVisualization

    TwoD
    Config
    ${MATPLOT_LIBRARIES}
)