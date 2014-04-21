cmake_minimum_required(VERSION 2.8.3)
project(genidlcpp)
find_package(catkin REQUIRED COMPONENTS genmsg rclcpp rclcpp_impl_ospl)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

catkin_package(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  CATKIN_DEPENDS genmsg rclcpp rclcpp_impl_ospl
  CFG_EXTRAS genidlcpp-extras.cmake
)

add_subdirectory(scripts)

file(WRITE ${CATKIN_DEVEL_PREFIX}/${GENMSG_LANGS_DESTINATION}/genidlcpp "IDLcpp")
install(FILES ${CATKIN_DEVEL_PREFIX}/${GENMSG_LANGS_DESTINATION}/genidlcpp
  DESTINATION ${GENMSG_LANGS_DESTINATION})

catkin_python_setup()

install(DIRECTORY include/genidlcpp/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
