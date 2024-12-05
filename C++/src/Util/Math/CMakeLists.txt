add_library(
    Mathematics
        Dynamics.cpp

)

target_link_libraries(
    Mathematics

    ${EIGEN3_LIBRARIES}
)