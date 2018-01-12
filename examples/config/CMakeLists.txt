add_custom_command(OUTPUT copyConfig POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/config/config.txt" "${EEROS_BINARY_DIR}/examples/config")

add_executable(configExample configExample.cpp copyConfig)
target_link_libraries(configExample eeros ${EEROS_LIBS})

