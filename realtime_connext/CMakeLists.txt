cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(realtime_connext)


find_package(nddscpp REQUIRED)
find_package(rttest REQUIRED)

add_definitions(${nddscpp_DEFINITIONS})

link_directories(${rttest_LIBRARY_DIR})
include_directories(${rttest_INCLUDE_DIRS} ${nddscpp_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS ${rttest_FLAGS})

add_library(poll_msg poll.cxx pollPlugin.cxx pollSupport.cxx)
target_link_libraries(poll_msg ${nddscpp_LIBRARIES})

add_executable(poll_publisher poll_publisher.cxx)
target_link_libraries(poll_publisher poll_msg ${rttest_LIBRARIES})

add_executable(poll_subscriber poll_subscriber.cxx)
target_link_libraries(poll_subscriber poll_msg ${rttest_LIBRARIES})

find_package(Threads)
if(NOT Threads_FOUND)
  message(WARNING "pthread not found. Intra-process pub/sub example will not be built")
elseif (Threads_FOUND)
  add_executable(poll_intraproc poll_intraproc.cxx)
  target_link_libraries(poll_intraproc poll_msg ${rttest_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
endif()
