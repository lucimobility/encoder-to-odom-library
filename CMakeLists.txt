cmake_minimum_required(VERSION 3.8)
project(encoder_to_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(encoder_to_odom 
    src/odometry.cpp
)

target_include_directories(encoder_to_odom PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

 install (TARGETS encoder_to_odom DESTINATION ${CMAKE_INSTALL_LIBDIR})