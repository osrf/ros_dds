cmake_minimum_required(VERSION 2.8.3)
project(rclcpp_examples)

find_package(catkin REQUIRED COMPONENTS rclcpp std_msgs message_generation)

include_directories(${catkin_INCLUDE_DIRS})

add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

generate_messages()

catkin_package()

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker ${catkin_EXPORTED_TARGETS})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener ${catkin_EXPORTED_TARGETS})

add_executable(in_process src/in_process.cpp)
target_link_libraries(in_process ${catkin_LIBRARIES})
add_dependencies(in_process ${catkin_EXPORTED_TARGETS})

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES} rclcpp_examples_dds_msgs)
add_dependencies(add_two_ints_server ${catkin_EXPORTED_TARGETS} rclcpp_examples_gencpp std_msgs_gencpp)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES} rclcpp_examples_dds_msgs)
add_dependencies(add_two_ints_client ${catkin_EXPORTED_TARGETS} rclcpp_examples_gencpp std_msgs_gencpp)

add_executable(parameter_client src/parameter_client.cpp)
target_link_libraries(parameter_client ${catkin_LIBRARIES} rclcpp_dds_msgs)
add_dependencies(parameter_client ${catkin_EXPORTED_TARGETS} rclcpp_gencpp)
