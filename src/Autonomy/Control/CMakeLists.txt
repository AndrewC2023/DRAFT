add_library(Control
    LQR.cpp
)

target_include_directories(Control
    PUBLIC
        ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(Control
    PUBLIC
        Eigen3::Eigen
)
