cmake_minimum_required(VERSION 2.8)

INCLUDE (CheckIncludeFiles)
include(FindBoost)

project(chatter)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idl)

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/idl/ccpp_StringMsg.h ${CMAKE_CURRENT_BINARY_DIR}/idl/StringMsgDcps.cpp ${CMAKE_CURRENT_BINARY_DIR}/idl/StringMsgDcps_impl.cpp ${CMAKE_CURRENT_BINARY_DIR}/idl/StringMsg.h ${CMAKE_CURRENT_BINARY_DIR}/idl/StringMsgSplDcps.cpp ${CMAKE_CURRENT_BINARY_DIR}/idl/StringMsg.cpp ${CMAKE_CURRENT_BINARY_DIR}/idl/StringMsgDcps.h ${CMAKE_CURRENT_BINARY_DIR}/idl/StringMsgDcps_impl.h ${CMAKE_CURRENT_BINARY_DIR}/idl/StringMsgSplDcps.h
                  COMMAND idlpp -l c++ -S ${CMAKE_CURRENT_SOURCE_DIR}/idl/StringMsg.idl
                  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idl
                  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/idl/StringMsg.idl)

add_custom_target(idl ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/idl/ccpp_StringMsg.h)
include_directories(cpp ${CMAKE_CURRENT_BINARY_DIR}/idl $ENV{OSPL_HOME}/include/dcps/C++/SACPP)
link_directories($ENV{OSPL_HOME}/lib)

add_library(StringMsgidl SHARED ${CMAKE_CURRENT_BINARY_DIR}/idl/StringMsgDcps.cpp ${CMAKE_CURRENT_BINARY_DIR}/idl/StringMsgDcps_impl.cpp ${CMAKE_CURRENT_BINARY_DIR}/idl/StringMsgSplDcps.cpp ${CMAKE_CURRENT_BINARY_DIR}/idl/StringMsg.cpp)


set(DDS_LIBS
cmjni
cmxml
commonserv
# This one needs other stuff
#dcpsccpp
dcpsgapi
dcpsisocpp
dcpsjni
dcpssacpp
dcpssac
dcpssaj
ddsconfparser
ddsconf
ddsdatabase
ddsi2
ddskernel
ddsloccollections
ddslocutil
ddsosnet
ddsos
ddsserialization
ddsuser
ddsutil
durability
spliced)

add_executable(publisher publisher.cc dds_robot.hh CheckStatus.cpp)
target_link_libraries(publisher StringMsgidl ${DDS_LIBS} boost_program_options)

add_executable(subscriber subscriber.cc dds_robot.hh CheckStatus.cpp)
target_link_libraries(subscriber StringMsgidl ${DDS_LIBS} boost_program_options)
