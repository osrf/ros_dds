cmake_minimum_required(VERSION 2.8.3)
project(foo)

find_package(catkin REQUIRED COMPONENTS genidlcpp)

message("catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS}")
find_package(opensplice REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${OPENSPLICE_INCLUDE_DIRS})

catkin_package()

add_executable(foo src/main.cpp)
add_dependencies(foo ${catkin_EXPORTED_TARGETS})
target_link_libraries(foo std_msgs_dds_msgs ${catkin_LIBRARIES} ${OPENSPLICE_LIBRARIES})
