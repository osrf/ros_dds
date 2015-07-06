cmake_minimum_required(VERSION 2.8)
project(realtime_c)

find_package(rttest REQUIRED)

find_package(opensplice REQUIRED)

# find_package only exposes the c++ include directories but not the c headers
include_directories(c /usr/include/opensplice/dcps/C/SAC ${OPENSPLICE_INCLUDE_DIRS} ${rttest_INCLUDE_DIRS})
link_directories(${OPENSPLICE_LIBRARY_DIRS} ${rttest_LIBRARY_DIR})

set(MSG_C_FILES c/LargeMsgSacDcps.c c/LargeMsgSplDcps.c)

add_executable(publisher rt_publisher.cpp ${MSG_C_FILES})
target_link_libraries(publisher ${OPENSPLICE_LIBRARIES} ${rttest_LIBRARIES} m stdc++)

add_executable(subscriber rt_subscriber.cpp ${MSG_C_FILES})
target_link_libraries(subscriber ${OPENSPLICE_LIBRARIES} ${rttest_LIBRARIES} m stdc++)

#add_executable(intraproc rt_intraproc.cpp ${MSG_C_FILES})
#target_link_libraries(intraproc ${OPENSPLICE_LIBRARIES} ${rttest_LIBRARIES} m stdc++)
