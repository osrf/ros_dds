install(
  FILES msg.idl.template srv.idl.template
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(
  PROGRAMS gen_idl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
