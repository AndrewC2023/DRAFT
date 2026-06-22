add_library(NonLinearModels
    QE3DofMissile.cpp
)

target_include_directories(NonLinearModels
    PUBLIC
        ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(NonLinearModels
    PUBLIC
        Dynamics
        Eigen3::Eigen
)
