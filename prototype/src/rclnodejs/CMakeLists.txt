cmake_minimum_required(VERSION 2.8.3)
project(rclnodejs)

find_package(catkin REQUIRED COMPONENTS rclcpp)

include_directories(include ${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-Wno-c++11-extensions")

catkin_package()

add_library(rclnodejs src/rclnodejs_addon.cpp src/rclnodejs/rclnodejs.cpp)
set_target_properties(rclnodejs
    PROPERTIES
        LINK_FLAGS "-undefined dynamic_lookup"
        OUTPUT_NAME rclnodejs.node
        PREFIX ""
        SUFFIX ""
)
target_link_libraries(rclnodejs ${catkin_LIBRARIES})
add_dependencies(rclnodejs ${catkin_EXPORTED_TARGETS})
