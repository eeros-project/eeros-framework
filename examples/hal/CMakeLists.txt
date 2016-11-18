
include_directories(control/)
include_directories(safety/)
include_directories(sequences/)


#set (CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-as-needed")

add_executable(parserTest parserTest.cpp
			  control/ParserTestControlSystem.cpp
			  safety/ParserTestSafetyProperties.cpp
			  sequences/ParserTestMainSequence.cpp
			)
			
			
target_link_libraries(parserTest eeros ${CMAKE_DL_LIBS} ucl)

