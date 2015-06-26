cmake_minimum_required(VERSION 2.8)
project(realtime)
set (CMAKE_CXX_FLAGS "-std=c++11")

find_package(opensplice REQUIRED)
include_directories(${OPENSPLICE_INCLUDE_DIRS})

ospl_add_idl(idl/LargeMsg.idl)

add_executable(dds_lms dds_lms.cpp)
target_link_libraries(dds_lms ${OPENSPLICE_LIBRARIES})

add_executable(dds_lmp dds_lmp.cpp)
target_link_libraries(dds_lmp ${OPENSPLICE_LIBRARIES})

find_package(Threads)
if(NOT Threads_FOUND)
	message(WARNING "Pthread not found. Real-time intra-process pub/sub example will not be built.")
elseif(Threads_FOUND)
	add_executable(dds_intraproc dds_intraproc.cpp)
	target_link_libraries(dds_intraproc ${OPENSPLICE_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
endif()
