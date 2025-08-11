find_package(GTest REQUIRED)

add_executable(encoder_tests encoder_test.cpp)

target_link_libraries(encoder_tests PRIVATE GTest::gtest_main encoder_to_odom_library)

include(GoogleTest)

gtest_discover_tests(encoder_tests)