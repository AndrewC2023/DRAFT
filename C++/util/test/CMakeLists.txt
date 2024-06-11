add_executable(
    Test_Config

    ConfigTest.cpp
)

target_link_libraries(
    Test_Config

    Config
    ${YAMLCPP_LIBRARIES}
)