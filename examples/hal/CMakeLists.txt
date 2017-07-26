add_custom_command(OUTPUT copyHalTest1Config POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/hal/HalTest1Config.json" "${EEROS_BINARY_DIR}/examples/hal")
add_custom_command(OUTPUT copyHalTest2CConfig POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/hal/HalTest2ConfigComedi.json" "${EEROS_BINARY_DIR}/examples/hal")
add_custom_command(OUTPUT copyHalTest2FConfig POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/hal/HalTest2ConfigFlink.json" "${EEROS_BINARY_DIR}/examples/hal")

add_executable(halTest1 HalTest1.cpp copyHalTest1Config)
target_link_libraries(halTest1 eeros ${EEROS_LIBS})

add_executable(halTest2 HalTest2.cpp copyHalTest2CConfig copyHalTest2FConfig)
target_link_libraries(halTest2 eeros ${EEROS_LIBS})



if ( roslib_FOUND )
	add_executable(rosNodeTalker RosNodeTalker.cpp)
	target_link_libraries(rosNodeTalker eeros ${EEROS_LIBS})
	
	add_custom_command(OUTPUT copyHalTest3Config POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/hal/HalTest3ConfigRos.json" "${EEROS_BINARY_DIR}/examples/hal")
	add_custom_command(OUTPUT HalTest3RosReadMe.txt POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/hal/HalTest3RosReadMe.txt" "${EEROS_BINARY_DIR}/examples/hal")
	add_executable(halTest3Ros HalTest3Ros.cpp copyHalTest3Config)
	target_link_libraries(halTest3Ros eeros ${EEROS_LIBS})
endif()