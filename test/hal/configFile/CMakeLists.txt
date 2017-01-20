
message("ext libs: ${EXTERNAL_LIBS}")

add_custom_command(OUTPUT copyConfig POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/test/hal/configFile/loadConfig.json" "${EEROS_BINARY_DIR}/test/hal/configFile")

add_executable(loadConfigFile loadConfigFile.cpp copyConfig)
target_link_libraries(loadConfigFile ${EXTERNAL_LIBS} eeros ucl dl gtest_main)
add_test(NAME loadConfigFile_test COMMAND loadConfigFile)