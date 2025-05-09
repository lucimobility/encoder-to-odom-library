cmake_minimum_required(VERSION 3.8)
project(encoder_to_odom CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(encoder_to_odom 
    src/odometry.cpp
)

target_include_directories(encoder_to_odom PUBLIC include
)

set_target_properties(encoder_to_odom PROPERTIES PUBLIC_HEADER "include/odometry.h")

install (TARGETS encoder_to_odom)

# Tests
if(NOT DEFINED NO_TESTS OR NOT NO_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()