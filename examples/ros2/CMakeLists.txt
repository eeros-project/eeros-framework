
if (rclcpp_FOUND AND USE_ROS2)
  set(targets "")

  add_executable(rosNodeTalker RosNodeTalker.cpp)
  target_link_libraries(rosNodeTalker eeros ${EEROS_LIBS} ${ROS_LIBRARIES} )
  list(APPEND targets rosNodeTalker)

  set(files "")	
  add_custom_command(OUTPUT copyHalTest2Config POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/ros2/RosTest2Config.json" "${EEROS_BINARY_DIR}/examples/ros2")
  list(APPEND files ${EEROS_BINARY_DIR}/examples/ros2/RosTest2Config.json)
  add_custom_command(OUTPUT copyHalTest3Config POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/ros2/HalConfigRos.json" "${EEROS_BINARY_DIR}/examples/ros2")
  list(APPEND files ${EEROS_BINARY_DIR}/examples/ros2/HalConfigRos.json)

  add_executable(rosTest1 RosTest1.cpp)
  target_link_libraries(rosTest1 eeros ${EEROS_LIBS} ${ROS_LIBRARIES})
  list(APPEND targets rosTest1)

  add_executable(rosTest2 RosTest2.cpp copyHalTest2Config copyHalTest3Config)
  target_link_libraries(rosTest2 eeros ${EEROS_LIBS} ${ROS_LIBRARIES})
  list(APPEND targets rosTest2)

  if(INSTALL_EXAMPLES)
    install(TARGETS ${targets} RUNTIME DESTINATION examples/ros2)
    install(FILES ${files} DESTINATION examples/ros2)
  endif()
endif()
