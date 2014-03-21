set(GENJSONCPP_PP_BIN "idlpp")
@[if DEVELSPACE]@
# bin and template dir variables in develspace
set(GENJSONCPP_BIN "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/gen_json_cpp.py")
set(GENJSONCPP_TEMPLATE_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/scripts")
@[else]@
# bin and template dir variables in installspace
set(GENJSONCPP_BIN "${genjsoncpp_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)/gen_json_cpp.py")
set(GENJSONCPP_TEMPLATE_DIR "${genjsoncpp_DIR}/..")
@[end if]@

# Generate .msg -> .h
# The generated .h files should be added ALL_GEN_OUTPUT_FILES_jsoncpp
macro(_generate_msg_jsoncpp ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  file(MAKE_DIRECTORY ${ARG_GEN_OUTPUT_DIR}/json)

  #Create input and output filenames
  get_filename_component(MSG_NAME ${ARG_MSG} NAME)
  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)

  set(IDL_DEPS "")
  foreach(dep ${ARG_MSG_DEPS})
    get_filename_component(dep_name ${dep} NAME_WE)
    get_filename_component(dep_pkg ${dep} PATH)
    get_filename_component(dep_pkg ${dep_pkg} PATH)
    get_filename_component(dep_pkg ${dep_pkg} NAME)
    list(APPEND IDL_DEPS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${dep_pkg}/dds_idl/${dep_name}_.idl")
    list(APPEND ${ARG_PKG}_MSG_DEPENDENCIES "${dep_pkg}")
  endforeach()

  set(COMMAND "${GENJSONCPP_PP_BIN}" -I "@(CATKIN_DEVEL_PREFIX)/@(CATKIN_GLOBAL_SHARE_DESTINATION)" -S -l cpp -o dds-types -d "${ARG_GEN_OUTPUT_DIR}/dds_impl" "${IDL_INPUT_FILE}")

  string(REPLACE ";" " " COMMAND_STR "${COMMAND}")
  #message("${COMMAND_STR}")

  add_custom_command(OUTPUT ${GEN_OUTPUT_FILES}
    DEPENDS ${IDL_DEPS} ${IDL_INPUT_FILE}
    COMMAND ${COMMAND}
    COMMENT "Generating DDS C++ code from ${ARG_PKG}/${MSG_SHORT_NAME}_.idl"
    )
  list(APPEND ALL_GEN_OUTPUT_FILES_jsoncpp ${GEN_OUTPUT_FILES})

  genjsoncpp_append_include_dirs()

  set(GEN_OUTPUT_FILES
    "${ARG_GEN_OUTPUT_DIR}/dds_impl/${MSG_SHORT_NAME}_convert.h"
  )

  set(COMMAND ${GENJSONCPP_BIN} ${ARG_MSG} ${ARG_PKG} -o "${ARG_GEN_OUTPUT_DIR}/dds_impl" -e ${GENJSONCPP_TEMPLATE_DIR})

  string(REPLACE ";" " " COMMAND_STR "${COMMAND}")
  #message("${COMMAND_STR}")

  assert(CATKIN_ENV)
  add_custom_command(OUTPUT ${GEN_OUTPUT_FILES}
    DEPENDS ${GENJSONCPP_BIN} ${ARG_MSG} "${GENJSONCPP_TEMPLATE_DIR}/msg_convert.h.template"
    COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${COMMAND}
    COMMENT "Generating conversion code between ROS and DDS message instances for ${ARG_PKG}/${MSG_NAME}"
    )
  list(APPEND ALL_GEN_OUTPUT_FILES_jsoncpp ${GEN_OUTPUT_FILES})
endmacro()

#genidl uses the same program to generate srv and msg files, so call the same macro
macro(_generate_srv_jsoncpp ARG_PKG ARG_SRV ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  #_generate_msg_jsoncpp(${ARG_PKG} ${ARG_SRV} "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" ${ARG_GEN_OUTPUT_DIR})
endmacro()

macro(_generate_module_jsoncpp ARG_PKG ARG_GEN_OUTPUT_DIR)
  include_directories(BEFORE ${genjsoncpp_INCLUDE_DIRS})
  if(${ARG_PKG}_MSG_DEPENDENCIES)
    list(REMOVE_DUPLICATES ${ARG_PKG}_MSG_DEPENDENCIES)
  endif()
  foreach(dep ${${ARG_PKG}_MSG_DEPENDENCIES})
    include_directories(BEFORE ${CATKIN_DEVEL_PREFIX}/include/${dep}/dds_impl)
  endforeach()
  add_library(${PROJECT_NAME}_dds_msgs ${ARGN})
  target_link_libraries(${PROJECT_NAME}_dds_msgs ${OPENSPLICE_LIBRARIES})
  list(APPEND ${PROJECT_NAME}_LIBRARIES ${PROJECT_NAME}_dds_msgs)  # This won't work until catkin supports it
endmacro()

set(genjsoncpp_INSTALL_DIR include)

macro(genjsoncpp_append_include_dirs)
  if(NOT genjsoncpp_APPENDED_INCLUDE_DIRS)
    # make sure we can find generated messages and that they overlay all other includes
    include_directories(BEFORE ${CATKIN_DEVEL_PREFIX}/${genjsoncpp_INSTALL_DIR})
    # pass the include directory to catkin_package()
    list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/${genjsoncpp_INSTALL_DIR})
    set(genjsoncpp_APPENDED_INCLUDE_DIRS TRUE)
  endif()
endmacro()
