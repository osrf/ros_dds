cmake_minimum_required(VERSION 2.8.3)
project(rclcpp)

find_package(opensplice REQUIRED)
find_package(catkin REQUIRED COMPONENTS genidlcpp)

find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(include ${OPENSPLICE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++0x")

catkin_package(
    INCLUDE_DIRS include ${OPENSPLICE_INCLUDE_DIRS}
    LIBRARIES rclcpp ${OPENSPLICE_LIBRARIES}
    CATKIN_DEPENDS genidlcpp
)

add_library(rclcpp src/rclcpp/rclcpp.cpp src/rclcpp/node/node.cpp)
target_link_libraries(rclcpp ${OPENSPLICE_LIBRARIES} ${Boost_LIBRARIES})
