
add_custom_command(OUTPUT copySimConfig POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/sim/SimConfig.json" "${EEROS_BINARY_DIR}/examples/sim")

add_executable(simExample simExample.cpp
			  control/SimControlSystem.cpp
			  safety/SimSafetyProperties.cpp
			  sequences/SimMainSequence.cpp
			  copySimConfig
			)
			
target_link_libraries(simExample eeros ucl ${CMAKE_DL_LIBS})
