add_executable(QETrials
    main.cpp
    QE3DofMissileGainScheduledLQR.cpp
    QE3DofMissileNDI.cpp
)

target_link_libraries(QETrials
    PRIVATE
        Control
        NonLinearModels
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)

target_include_directories(QETrials
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)
