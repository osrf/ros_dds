cmake_minimum_required(VERSION 2.8)
project(topic)

find_package(opensplice REQUIRED COMPONENTS C)

include_directories(${OPENSPLICE_INCLUDE_DIRS})

ospl_add_idl(Chat.idl)
add_executable(topic main.c)
target_link_libraries(topic ${OPENSPLICE_LIBRARIES})
