cmake_minimum_required(VERSION 2.8.3)
project(rclcpp)

find_package(opensplice REQUIRED)
find_package(catkin REQUIRED COMPONENTS genidlcpp)

ospl_add_idl(idl/ROSMsg.idl)

include_directories(include ${OPENSPLICE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${OPENSPLICE_LIBRARY_DIRS})

add_library(rclcpp src/rclcpp/rclcpp.cpp src/rclcpp/node/node.cpp src/rclcpp/subscription/subscription.cpp)
target_link_libraries(rclcpp ${OPENSPLICE_LIBRARIES})

add_executable(example example/src/main.cpp)
target_link_libraries(example rclcpp ${OPENSPLICE_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(example ${catkin_EXPORTED_TARGETS})
