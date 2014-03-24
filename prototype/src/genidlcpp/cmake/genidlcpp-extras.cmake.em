find_package(opensplice REQUIRED)
include_directories(${OPENSPLICE_INCLUDE_DIRS})

set(GENIDLCPP_PP_BIN "idlpp")
@[if DEVELSPACE]@
# bin and template dir variables in develspace
set(GENIDLCPP_BIN "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/gen_idl_cpp_convert.py")
set(GENIDLCPP_TEMPLATE_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/scripts")
@[else]@
# bin and template dir variables in installspace
set(GENIDLCPP_BIN "${genidlcpp_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)/gen_idl_cpp_convert.py")
set(GENIDLCPP_TEMPLATE_DIR "${genidlcpp_DIR}/..")
@[end if]@

# Generate .idl -> dds code generation and _convert.h
# The generated files should be added to ALL_GEN_OUTPUT_FILES_idlcpp
macro(_generate_msg_idlcpp ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  #message("msg ${ARG_PKG} ${ARG_MSG} ${ARG_IFLAGS} ${ARG_MSG_DEPS}")

  # generate dds code for message
  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)
  _generate_msg_idlcpp_dds(${ARG_PKG} ${MSG_SHORT_NAME} "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" ${ARG_GEN_OUTPUT_DIR})
  # generate _convert.h for message
  _generate_msg_idlcpp_convert(${ARG_PKG} ${ARG_MSG} "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" ${ARG_GEN_OUTPUT_DIR})
endmacro()

# Generate .idl -> dds code generation and _convert.h
# The generated files should be added to ALL_GEN_OUTPUT_FILES_idlcpp
macro(_generate_srv_idlcpp ARG_PKG ARG_SRV ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  #message("srv ${ARG_PKG} ${ARG_SRV} ${ARG_IFLAGS} ${ARG_MSG_DEPS}")

  # generate dds code for service as well as request and response messages
  get_filename_component(SRV_SHORT_NAME ${ARG_SRV} NAME_WE)
  _generate_msg_idlcpp_dds(${ARG_PKG} "${SRV_SHORT_NAME}" "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" ${ARG_GEN_OUTPUT_DIR})
  _generate_msg_idlcpp_dds(${ARG_PKG} "${SRV_SHORT_NAME}Request" "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" ${ARG_GEN_OUTPUT_DIR})
  _generate_msg_idlcpp_dds(${ARG_PKG} "${SRV_SHORT_NAME}Response" "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" ${ARG_GEN_OUTPUT_DIR})
  # generate _convert.h for request and response
  _generate_msg_idlcpp_convert(${ARG_PKG} ${ARG_SRV} "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" ${ARG_GEN_OUTPUT_DIR})
endmacro()

macro(_generate_msg_idlcpp_dds ARG_PKG MSG_SHORT_NAME ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  #message("msg_dds ${ARG_PKG} ${MSG_SHORT_NAME} ${ARG_IFLAGS} ${ARG_MSG_DEPS}")

  set(IDL_INPUT_FILE "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${ARG_PKG}/dds_idl/${MSG_SHORT_NAME}_.idl")
  set(GEN_OUTPUT_FILES
    "${ARG_GEN_OUTPUT_DIR}/dds_impl/${MSG_SHORT_NAME}_.h"
    "${ARG_GEN_OUTPUT_DIR}/dds_impl/${MSG_SHORT_NAME}_.cpp"
    "${ARG_GEN_OUTPUT_DIR}/dds_impl/${MSG_SHORT_NAME}_Dcps.h"
    "${ARG_GEN_OUTPUT_DIR}/dds_impl/${MSG_SHORT_NAME}_Dcps.cpp"
    "${ARG_GEN_OUTPUT_DIR}/dds_impl/${MSG_SHORT_NAME}_Dcps_impl.h"
    "${ARG_GEN_OUTPUT_DIR}/dds_impl/${MSG_SHORT_NAME}_Dcps_impl.cpp"
    "${ARG_GEN_OUTPUT_DIR}/dds_impl/${MSG_SHORT_NAME}_SplDcps.h"
    "${ARG_GEN_OUTPUT_DIR}/dds_impl/${MSG_SHORT_NAME}_SplDcps.cpp"
    "${ARG_GEN_OUTPUT_DIR}/dds_impl/ccpp_${MSG_SHORT_NAME}_.h"
  )

  set(IDL_DEPS "")
  foreach(dep ${ARG_MSG_DEPS})
    get_filename_component(dep_name ${dep} NAME_WE)
    get_filename_component(dep_pkg ${dep} PATH)
    get_filename_component(dep_pkg ${dep_pkg} PATH)
    get_filename_component(dep_pkg ${dep_pkg} NAME)
    list(APPEND IDL_DEPS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${dep_pkg}/dds_idl/${dep_name}_.idl")
    list(APPEND ${ARG_PKG}_MSG_DEPENDENCIES "${dep_pkg}")
  endforeach()

  set(COMMAND "${GENIDLCPP_PP_BIN}" -I "@(CATKIN_DEVEL_PREFIX)/@(CATKIN_GLOBAL_SHARE_DESTINATION)" -S -l cpp -o dds-types -d "${ARG_GEN_OUTPUT_DIR}/dds_impl" "${IDL_INPUT_FILE}")

  string(REPLACE ";" " " COMMAND_STR "${COMMAND}")
  #message("${COMMAND_STR}")

  file(MAKE_DIRECTORY ${ARG_GEN_OUTPUT_DIR}/dds_impl)

  add_custom_command(OUTPUT ${GEN_OUTPUT_FILES}
    DEPENDS ${IDL_DEPS} ${IDL_INPUT_FILE}
    COMMAND ${COMMAND}
    COMMENT "Generating DDS C++ code from ${ARG_PKG}/${MSG_SHORT_NAME}_.idl"
    )
  list(APPEND ALL_GEN_OUTPUT_FILES_idlcpp ${GEN_OUTPUT_FILES})

  genidlcpp_append_include_dirs()
endmacro()

macro(_generate_msg_idlcpp_convert ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  #message("msg_convert ${ARG_PKG} ${ARG_MSG} ${ARG_IFLAGS} ${ARG_MSG_DEPS}")

  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)
  set(GEN_OUTPUT_FILES
    "${ARG_GEN_OUTPUT_DIR}/dds_impl/${MSG_SHORT_NAME}_convert.h"
  )

  set(COMMAND ${GENIDLCPP_BIN} ${ARG_MSG} ${ARG_PKG} -o "${ARG_GEN_OUTPUT_DIR}/dds_impl" -e ${GENIDLCPP_TEMPLATE_DIR})

  string(REPLACE ";" " " COMMAND_STR "${COMMAND}")
  #message("${COMMAND_STR}")

  get_filename_component(MSG_NAME ${ARG_MSG} NAME)

  assert(CATKIN_ENV)
  add_custom_command(OUTPUT ${GEN_OUTPUT_FILES}
    DEPENDS ${GENIDLCPP_BIN} ${ARG_MSG} "${GENIDLCPP_TEMPLATE_DIR}/msg_convert.h.template"
    COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${COMMAND}
    COMMENT "Generating conversion code between ROS and DDS message instances for ${ARG_PKG}/${MSG_NAME}"
    )
  list(APPEND ALL_GEN_OUTPUT_FILES_idlcpp ${GEN_OUTPUT_FILES})

  genidlcpp_append_include_dirs()
endmacro()

macro(_generate_module_idlcpp ARG_PKG ARG_GEN_OUTPUT_DIR)
  include_directories(BEFORE ${genidlcpp_INCLUDE_DIRS})
  if(${ARG_PKG}_MSG_DEPENDENCIES)
    list(REMOVE_DUPLICATES ${ARG_PKG}_MSG_DEPENDENCIES)
  endif()
  set(dds_impl_libs "")
  foreach(dep ${${ARG_PKG}_MSG_DEPENDENCIES})
    include_directories(BEFORE ${CATKIN_DEVEL_PREFIX}/include/${dep}/dds_impl)
    list(APPEND dds_impl_libs ${dep}_dds_msgs)
  endforeach()
  if(NOT "${ARGN}" STREQUAL "")
    add_library(${PROJECT_NAME}_dds_msgs ${ARGN})
    target_link_libraries(${PROJECT_NAME}_dds_msgs ${dds_impl_libs} ${OPENSPLICE_LIBRARIES})
    list(APPEND ${PROJECT_NAME}_LIBRARIES ${PROJECT_NAME}_dds_msgs)
  endif()
endmacro()

set(genidlcpp_INSTALL_DIR include)

macro(genidlcpp_append_include_dirs)
  if(NOT genidlcpp_APPENDED_INCLUDE_DIRS)
    # make sure we can find generated messages and that they overlay all other includes
    include_directories(BEFORE ${CATKIN_DEVEL_PREFIX}/${genidlcpp_INSTALL_DIR})
    # pass the include directory to catkin_package()
    list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/${genidlcpp_INSTALL_DIR})
    set(genidlcpp_APPENDED_INCLUDE_DIRS TRUE)
  endif()
endmacro()
