cmake_minimum_required(VERSION 2.8.3)
project(rclcpp)

find_package(catkin REQUIRED COMPONENTS rclcpp_impl_ospl)

include_directories(include ${catkin_INCLUDE_DIRS})

# For older versions of clang, libstdc++ is used by default and we need to
# explicitly request libc++ instead.
if(APPLE)
  set(CMAKE_CXX_FLAGS "-std=c++0x -stdlib=libc++")
else()
  set(CMAKE_CXX_FLAGS "-std=c++0x")
endif ()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES rclcpp
    CFG_EXTRAS rclcpp-extras.cmake
)

add_library(rclcpp
    src/rclcpp/rclcpp.cpp
    src/rclcpp/node/node.cpp
    src/rclcpp/publisher/publisher.cpp
    src/rclcpp/subscription/subscription.cpp
)
target_link_libraries(rclcpp ${OPENSPLICE_LIBRARIES})

install(TARGETS rclcpp
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/rclcpp/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
