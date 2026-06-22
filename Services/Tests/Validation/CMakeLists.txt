add_executable(QE3DofMissileValidation
    QE3DofMissileValidation.cpp
    QETrials/QE3DofMissileNDI.cpp
)

target_link_libraries(QE3DofMissileValidation
    PRIVATE
        NonLinearModels
        Eigen3::Eigen
)

target_include_directories(QE3DofMissileValidation
    PRIVATE
        QETrials
)

add_subdirectory(QETrials)
