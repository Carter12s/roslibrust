cmake_minimum_required(VERSION 3.16.0)
project(test_package)

add_executable(test_main
    src/test_main.cpp
)
target_include_directories(test_main PRIVATE
    include
    /opt/ros/noetic/include
)