cmake_minimum_required(VERSION 2.8)
project(ros_dds)

include(FindBoost)
find_package(opensplice REQUIRED)
include_directories(cpp ${OPENSPLICE_INCLUDE_DIRS})
link_directories(${OPENSPLICE_LIBRARY_DIRS})

ospl_add_idl(idl/StringMsg.idl)

add_executable(publisher publisher.cc dds_robot.hh CheckStatus.cpp)
target_link_libraries(publisher ${OPENSPLICE_LIBRARIES} boost_program_options)

add_executable(subscriber subscriber.cc dds_robot.hh CheckStatus.cpp)
target_link_libraries(subscriber ${OPENSPLICE_LIBRARIES} boost_program_options)
