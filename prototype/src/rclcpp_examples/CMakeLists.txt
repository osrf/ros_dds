cmake_minimum_required(VERSION 2.8.3)
project(rclcpp_examples)

find_package(catkin REQUIRED COMPONENTS rclcpp std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_message_files(DIRECTORY srv FILES AddTwoIntsRequest.msg AddTwoIntsResponse.msg)
generate_messages()

catkin_package()

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker ${catkin_EXPORTED_TARGETS})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener ${catkin_EXPORTED_TARGETS})

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES} std_msgs_dds_msgs)
add_dependencies(add_two_ints_server ${catkin_EXPORTED_TARGETS})

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES} std_msgs_dds_msgs)
add_dependencies(add_two_ints_client ${catkin_EXPORTED_TARGETS})
