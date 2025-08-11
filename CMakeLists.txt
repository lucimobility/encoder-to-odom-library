cmake_minimum_required(VERSION 3.8)
project(encoder_to_odom_library CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(encoder_to_odom_library 
    src/odometry.cpp
)

target_include_directories(encoder_to_odom_library PUBLIC include
)

set_target_properties(encoder_to_odom_library PROPERTIES PUBLIC_HEADER "include/odometry.h")

install (TARGETS encoder_to_odom_library)

# Tests
if(NOT DEFINED NO_TESTS OR NOT NO_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()