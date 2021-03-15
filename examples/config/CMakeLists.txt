add_custom_command(OUTPUT copyConfig POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/config/config.txt" "${EEROS_BINARY_DIR}/examples/config")

add_executable(configExample ConfigExample.cpp copyConfig)
target_link_libraries(configExample eeros ${EEROS_LIBS})

if(INSTALL_EXAMPLES)
  install(TARGETS configExample RUNTIME DESTINATION examples/config)
  install(FILES ${EEROS_BINARY_DIR}/examples/config/config.txt DESTINATION examples/config)
endif()

