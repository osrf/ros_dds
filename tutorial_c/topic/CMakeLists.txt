cmake_minimum_required(VERSION 2.8)
project(topic)

find_package(opensplice REQUIRED)
# find_package only exposes the c++ include directories but not the c headers
include_directories(c /usr/include/opensplice/dcps/C/SAC ${OPENSPLICE_INCLUDE_DIRS})
link_directories(${OPENSPLICE_LIBRARY_DIRS})

add_executable(topic main.cc c/ChatSacDcps.c c/ChatSplDcps.c)
target_link_libraries(topic ${OPENSPLICE_LIBRARIES})
