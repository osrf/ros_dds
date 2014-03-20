cmake_minimum_required(VERSION 2.8.3)
project(rclcpp_examples)

find_package(catkin REQUIRED COMPONENTS rclcpp std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES} std_msgs_dds_msgs)
add_dependencies(talker ${catkin_EXPORTED_TARGETS})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES} std_msgs_dds_msgs)
add_dependencies(listener ${catkin_EXPORTED_TARGETS})
