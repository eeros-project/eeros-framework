
include_directories(control/)
include_directories(safety/)
include_directories(sequences/)


#set (CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-as-needed")

add_custom_command(OUTPUT copyParserTestConfig POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/hal/HALConfigComedi.json" "${EEROS_BINARY_DIR}/examples/hal")

add_executable(parserTest parserTest.cpp
			  control/ParserTestControlSystem.cpp
			  safety/ParserTestSafetyProperties.cpp
			  sequences/ParserTestMainSequence.cpp
			  copyParserTestConfig
			)
			
			
target_link_libraries(parserTest eeros ${EEROS_LIBS})

