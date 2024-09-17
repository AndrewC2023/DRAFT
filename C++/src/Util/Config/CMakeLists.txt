add_library(
    Config

        Config.hpp
        Config.cpp
)

target_link_libraries(
    Config

    yaml-cpp
)