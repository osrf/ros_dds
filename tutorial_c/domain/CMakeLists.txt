cmake_minimum_required(VERSION 2.8)
project(domain)

find_package(opensplice REQUIRED)
# find_package only exposes the c++ include directories but not the c headers
include_directories(/usr/include/opensplice/dcps/C/SAC ${OPENSPLICE_INCLUDE_DIRS})
link_directories(${OPENSPLICE_LIBRARY_DIRS})

add_executable(domain main.cc)
target_link_libraries(domain ${OPENSPLICE_LIBRARIES})
