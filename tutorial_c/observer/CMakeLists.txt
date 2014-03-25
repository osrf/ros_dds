cmake_minimum_required(VERSION 2.8)
project(observer)

find_package(opensplice REQUIRED)
# find_package only exposes the c++ include directories but not the c headers
include_directories(c /usr/include/opensplice/dcps/C/SAC ${OPENSPLICE_INCLUDE_DIRS})
link_directories(${OPENSPLICE_LIBRARY_DIRS})

add_executable(observer main.c)
target_link_libraries(observer ${OPENSPLICE_LIBRARIES})
