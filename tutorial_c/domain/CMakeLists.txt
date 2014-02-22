cmake_minimum_required(VERSION 2.8)
project(domain)

find_package(opensplice REQUIRED COMPONENTS C)
include_directories(${OPENSPLICE_INCLUDE_DIRS})

add_executable(domain main.c)
target_link_libraries(domain ${OPENSPLICE_LIBRARIES})
