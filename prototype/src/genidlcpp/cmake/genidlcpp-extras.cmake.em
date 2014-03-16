set(GENIDLCPP_PP_BIN "idlpp")

# Generate .idl -> .h
# The generated .h files should be added ALL_GEN_OUTPUT_FILES_idlcpp
macro(_generate_msg_idlcpp ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  file(MAKE_DIRECTORY ${ARG_GEN_OUTPUT_DIR}/dds_impl)

  #Create input and output filenames
  get_filename_component(MSG_NAME ${ARG_MSG} NAME)
  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)

  set(IDL_INPUT_FILE "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${ARG_PKG}/dds_idl/${MSG_SHORT_NAME}.idl")
  set(GEN_OUTPUT_FILE ${ARG_GEN_OUTPUT_DIR}/dds_impl/${MSG_SHORT_NAME}.h)

  set(IDL_DEPS "")
  foreach(dep ${ARG_MSG_DEPS})
    get_filename_component(dep_name ${dep} NAME_WE)
    get_filename_component(dep_pkg ${dep} PATH)
    get_filename_component(dep_pkg ${dep_pkg} PATH)
    get_filename_component(dep_pkg ${dep_pkg} NAME)
    list(APPEND IDL_DEPS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/${dep_pkg}/dds_idl/${dep_name}.idl")
  endforeach()

  set(COMMAND "${GENIDLCPP_PP_BIN}" -I "@(CATKIN_DEVEL_PREFIX)/@(CATKIN_GLOBAL_SHARE_DESTINATION)" -S -l cpp -o dds-types -d "${ARG_GEN_OUTPUT_DIR}/dds_impl" "${IDL_INPUT_FILE}")

  string(REPLACE ";" " " COMMAND_STR "${COMMAND}")
  message("${COMMAND_STR}")

  add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
    DEPENDS ${IDL_DEPS} ${IDL_INPUT_FILE}
    COMMAND ${COMMAND}
    COMMENT "Generating DDS C++ code from ${ARG_PKG}/${MSG_SHORT_NAME}.idl"
    )
  list(APPEND ALL_GEN_OUTPUT_FILES_idlcpp ${GEN_OUTPUT_FILE})

  genidlcpp_append_include_dirs()
endmacro()

#genidl uses the same program to generate srv and msg files, so call the same macro
macro(_generate_srv_idlcpp ARG_PKG ARG_SRV ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  #_generate_msg_idlcpp(${ARG_PKG} ${ARG_SRV} "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" ${ARG_GEN_OUTPUT_DIR})
endmacro()

macro(_generate_module_idlcpp)
  # the macros, they do nothing
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
