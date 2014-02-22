cmake_minimum_required(VERSION 2.8)
project(ros_dds)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(opensplice REQUIRED)
include_directories(${OPENSPLICE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

ospl_add_idl(idl/StringMsg.idl)

add_executable(publisher publisher.cc dds_robot.hh CheckStatus.cpp)
target_link_libraries(publisher ${OPENSPLICE_LIBRARIES} ${Boost_LIBRARIES})

add_executable(subscriber subscriber.cc dds_robot.hh CheckStatus.cpp)
target_link_libraries(subscriber ${OPENSPLICE_LIBRARIES} ${Boost_LIBRARIES})
