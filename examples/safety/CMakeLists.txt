add_custom_command(OUTPUT copySafetySystemConfig POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/safety/SafetySystemTestConfig.json" "${EEROS_BINARY_DIR}/examples/safety")

add_executable(safetySystemTest1 SafetySystemTest1.cpp copySafetySystemConfig)
target_link_libraries(safetySystemTest1 eeros ucl ${CMAKE_DL_LIBS} )

add_executable(safetySystemTest2 SafetySystemTest2.cpp)
target_link_libraries(safetySystemTest2 eeros pthread)
