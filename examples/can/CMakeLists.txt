
#if (roslib_FOUND AND USE_ROS)
#	add_executable(rosNodeTalker RosNodeTalker.cpp)
#	target_link_libraries(rosNodeTalker eeros ${EEROS_LIBS})
	
#	add_custom_command(OUTPUT copyHalTest2Config POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/ros/RosTest2Config.json" "${EEROS_BINARY_DIR}/examples/ros")
#	add_custom_command(OUTPUT copyHalTest3Config POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/ros/HalConfigRos.json" "${EEROS_BINARY_DIR}/examples/ros")

	add_executable(canExample CanExample.cpp)
	target_link_libraries(canExample eeros canopen ${EEROS_LIBS})

#	add_executable(rosTest1 RosTest1.cpp)
#	target_link_libraries(rosTest1 eeros ${EEROS_LIBS})
	
#	add_executable(rosTest2 RosTest2.cpp copyHalTest2Config)
#	target_link_libraries(rosTest2 eeros ${EEROS_LIBS})
#endif()