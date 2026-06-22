add_executable(LQRUnitTest
    LQRTest.cpp
)

target_link_libraries(LQRUnitTest
    PRIVATE
        Control
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(LQRUnitTest)
