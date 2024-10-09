include(${PROJECT_SOURCE_DIR}/cmake/target_management.cmake)

add_custom_command(OUTPUT copyConfig POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/config/config.txt" "${EEROS_BINARY_DIR}/examples/config")

eeros_add_target(configExample ConfigExample.cpp copyConfig)
