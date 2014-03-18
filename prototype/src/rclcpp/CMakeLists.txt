cmake_minimum_required(VERSION 2.8.3)
project(rclcpp)

find_package(catkin REQUIRED)

include_directories(include)

add_library(rclcpp src/rclcpp/rclcpp.cpp src/rclcpp/node/node.cpp)

add_executable(example src/main.cpp)
target_link_libraries(example rclcpp)
