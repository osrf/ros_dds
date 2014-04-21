cmake_minimum_required(VERSION 2.8.3)
project(rclcpp_impl_ospl)

find_package(opensplice REQUIRED)
find_package(catkin REQUIRED)

include_directories(include ${OPENSPLICE_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include ${OPENSPLICE_INCLUDE_DIRS}
    LIBRARIES ${OPENSPLICE_LIBRARIES}
)
