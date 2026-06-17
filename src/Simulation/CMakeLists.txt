add_library(Simulation
    DraftSim.cpp
)

target_include_directories(Simulation
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)