
message("ext libs: ${EXTERNAL_LIBS}")

add_executable(loadConfigFile loadConfigFile.cpp)
target_link_libraries(loadConfigFile ${EXTERNAL_LIBS} gtest_main)
add_test(NAME loadConfigFile_test COMMAND loadConfigFile)