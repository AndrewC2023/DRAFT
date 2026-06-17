add_library(Dynamics INTERFACE)

target_include_directories(Dynamics
    INTERFACE
        ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(Dynamics
    INTERFACE
        Eigen3::Eigen
)