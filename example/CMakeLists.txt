cmake_minimum_required(VERSION 2.8)
project(chatter)

find_package(opensplice REQUIRED)
include_directories(cpp ${OPENSPLICE_INCLUDE_DIRS})
link_directories(${OPENSPLICE_LIBRARY_DIRS})

ospl_add_idl(idl/Chat.idl idl/Chat2.idl)

add_executable(chatter cpp/src/Chatter.cpp cpp/src/CheckStatus.cpp)
target_link_libraries(chatter ${OPENSPLICE_LIBRARIES})

add_executable(messageboard cpp/src/MessageBoard.cpp cpp/src/multitopic.cpp cpp/src/CheckStatus.cpp)
target_link_libraries(messageboard ${OPENSPLICE_LIBRARIES})

add_executable(userload cpp/src/UserLoad.cpp cpp/src/CheckStatus.cpp)
target_link_libraries(userload ${OPENSPLICE_LIBRARIES})

