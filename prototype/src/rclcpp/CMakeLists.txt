cmake_minimum_required(VERSION 2.8.3)
project(rclcpp)

find_package(opensplice REQUIRED)
find_package(catkin REQUIRED COMPONENTS genidlcpp std_msgs message_generation)

include_directories(include ${OPENSPLICE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# For older versions of clang, libstdc++ is used by default and we need to
# explicitly request libc++ instead.
if(APPLE)
  set(CMAKE_CXX_FLAGS "-std=c++0x -stdlib=libc++")
else()
  set(CMAKE_CXX_FLAGS "-std=c++0x")
endif ()

add_service_files(
    DIRECTORY srv FILES
    ParameterServerGetString.srv
    ParameterServerSetString.srv

    ParameterServerGetInt64.srv
    ParameterServerSetInt64.srv

    ParameterServerGetBool.srv
    ParameterServerSetBool.srv

    ParameterServerHas.srv

    ParameterServerDelete.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    INCLUDE_DIRS include ${OPENSPLICE_INCLUDE_DIRS}
    LIBRARIES rclcpp ${OPENSPLICE_LIBRARIES}
    CATKIN_DEPENDS genidlcpp
    CFG_EXTRAS rclcpp-extras.cmake
)

add_library(rclcpp src/rclcpp/rclcpp.cpp src/rclcpp/node/node.cpp)
target_link_libraries(rclcpp ${OPENSPLICE_LIBRARIES} ${catkin_LIBRARIES} rclcpp_dds_msgs)
add_dependencies(rclcpp ${catkin_EXPORTED_TARGETS} std_msgs_gencpp rclcpp_gencpp std_msgs_dds_msgs_gencpp genidlcpp)

install(TARGETS rclcpp
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/rclcpp/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_executable(parameter_server src/rclcpp/parameter_server.cpp)
target_link_libraries(parameter_server ${catkin_LIBRARIES} rclcpp)
add_dependencies(parameter_server ${catkin_EXPORTED_TARGETS})
