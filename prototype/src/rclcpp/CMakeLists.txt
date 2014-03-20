cmake_minimum_required(VERSION 2.8.3)
project(rclcpp)

find_package(opensplice REQUIRED)
find_package(catkin REQUIRED)
find_package(Boost COMPONENTS thread system REQUIRED)

ospl_add_idl(idl/ROSMsg.idl)

include_directories(include ${OPENSPLICE_INCLUDE_DIRS})
link_directories(${OPENSPLICE_LIBRARY_DIRS})

add_library(rclcpp src/rclcpp/rclcpp.cpp src/rclcpp/node/node.cpp src/rclcpp/publisher/publisher.cpp src/rclcpp/subscription/subscription.cpp)
target_link_libraries(rclcpp ${OPENSPLICE_LIBRARIES} ${Boost_THREAD_LIBRARY})

add_executable(talker src/talker.cpp)
target_link_libraries(talker rclcpp ${OPENSPLICE_LIBRARIES} ${Boost_THREAD_LIBRARY})

add_executable(listener src/listener.cpp)
target_link_libraries(listener rclcpp ${OPENSPLICE_LIBRARIES} ${Boost_THREAD_LIBRARY})
