@[if DEVELSPACE]@
# bin and template dir variables in develspace
set(GENIDL_BIN "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/gen_idl.py")
set(GENIDL_TEMPLATE_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/scripts")
@[else]@
# bin and template dir variables in installspace
set(GENIDL_BIN "${genidl_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)/gen_idl.py")
set(GENIDL_TEMPLATE_DIR "${genidl_DIR}/..")
@[end if]@

# Generate .msg -> .idl
# The generated .idl files should be added ALL_GEN_OUTPUT_FILES_idl
macro(_generate_msg_idl ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  set(GEN_OUTPUT_DIR "${ARG_GEN_OUTPUT_DIR}/dds_idl")
  file(MAKE_DIRECTORY ${GEN_OUTPUT_DIR})

  #Create input and output filenames
  get_filename_component(MSG_NAME ${ARG_MSG} NAME)
  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)

  set(MSG_GENERATED_NAME ${MSG_SHORT_NAME}.idl)
  set(GEN_OUTPUT_FILE ${GEN_OUTPUT_DIR}/${MSG_GENERATED_NAME})

  assert(CATKIN_ENV)
  add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
    DEPENDS ${GENIDL_BIN} ${ARG_MSG} ${ARG_MSG_DEPS} "${GENIDL_TEMPLATE_DIR}/msg.idl.template" ${ARGN}
    COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENIDL_BIN} ${ARG_MSG}
    ${ARG_IFLAGS}
    -p ${ARG_PKG}
    -o ${GEN_OUTPUT_DIR}
    -e ${GENIDL_TEMPLATE_DIR}
    COMMENT "Generating IDL file from ${ARG_PKG}/${MSG_NAME}"
    )
  list(APPEND ALL_GEN_OUTPUT_FILES_idl ${GEN_OUTPUT_FILE})
endmacro()

#genidl uses the same program to generate srv and msg files, so call the same macro
macro(_generate_srv_idl ARG_PKG ARG_SRV ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  #_generate_msg_idl(${ARG_PKG} ${ARG_SRV} "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" ${ARG_GEN_OUTPUT_DIR} "${GENIDL_TEMPLATE_DIR}/srv.idl.template")
endmacro()

macro(_generate_module_idl)
  # the macros, they do nothing
endmacro()

set(genidl_INSTALL_DIR share)
