cmake_minimum_required(VERSION 2.8)
project(large_message_cpp)

find_package(opensplice REQUIRED)
include_directories(${OPENSPLICE_INCLUDE_DIRS})

ospl_add_idl(idl/LargeMsg.idl)

add_executable(dds_lms dds_lms.cpp)
target_link_libraries(dds_lms ${OPENSPLICE_LIBRARIES})

add_executable(dds_lmp dds_lmp.cpp)
target_link_libraries(dds_lmp ${OPENSPLICE_LIBRARIES})

find_package(catkin)

if(catkin_FOUND)
    find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp)

    add_message_files(DIRECTORY msg FILES LargeMessage.msg)
    generate_messages(DEPENDENCIES std_msgs)

    catkin_package()

    include_directories(${catkin_INCLUDE_DIRS})

    add_executable(ros_lms ros_lms.cpp)
    target_link_libraries(ros_lms ${catkin_LIBRARIES})

    add_executable(ros_lmp ros_lmp.cpp)
    target_link_libraries(ros_lmp ${catkin_LIBRARIES})
else()
    message("catkin not found, skipping ROS comparison nodes")
endif()
