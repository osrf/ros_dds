install(
  FILES msg_convert.h.template
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(
  PROGRAMS gen_idl_cpp_convert.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
