
add_custom_command(OUTPUT copySafetySystemConfig POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/safety/SafetySystemTestConfig.json" "${EEROS_BINARY_DIR}/examples/safety")

add_executable(safetySystemTest1 SafetySystemTest1.cpp copySafetySystemConfig)
target_link_libraries(safetySystemTest1 eeros pthread)
