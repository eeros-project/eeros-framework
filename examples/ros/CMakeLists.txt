 
if (roslib_FOUND AND USE_ROS)
  set(targets "")

  find_package( std_msgs REQUIRED )
  list(APPEND MSG_LIBRARIES ${std_msgs_LIBRARIES})

  find_package( sensor_msgs REQUIRED )
  list(APPEND MSG_LIBRARIES ${sensor_msgs_LIBRARIES})

  include_directories(${rclcpp_INCLUDE_DIRS})
  include_directories(${rcl_interfaces_INCLUDE_DIRS})
  include_directories(${std_msgs_INCLUDE_DIRS})
  include_directories(${sensor_msgs_INCLUDE_DIRS})

  add_executable(rosNodeTalker RosNodeTalker.cpp)
  target_link_libraries(rosNodeTalker eeros ${EEROS_LIBS} ${ROS_LIBRARIES} ${MSG_LIBRARIES})

  list(APPEND targets rosNodeTalker)

  set(files "")
  add_custom_command(OUTPUT copyHalTest2Config POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/ros/RosTest2Config.json" "${EEROS_BINARY_DIR}/examples/ros")
  list(APPEND files ${EEROS_BINARY_DIR}/examples/ros/RosTest2Config.json)
  add_custom_command(OUTPUT copyHalTest3Config POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/ros/HalConfigRos.json" "${EEROS_BINARY_DIR}/examples/ros")
  list(APPEND files ${EEROS_BINARY_DIR}/examples/ros/HalConfigRos.json)

  add_executable(rosExample RosExample.cpp copyHalTest3Config)
  target_link_libraries(rosExample eeros ${EEROS_LIBS} ${ROS_LIBRARIES}  ${MSG_LIBRARIES})
  list(APPEND targets rosExample)

  add_executable(rosTest1 RosTest1.cpp)
  target_link_libraries(rosTest1 eeros ${EEROS_LIBS} ${ROS_LIBRARIES}  ${MSG_LIBRARIES})
  list(APPEND targets rosTest1)

  add_executable(rosTest2 RosTest2.cpp copyHalTest2Config)
  target_link_libraries(rosTest2 eeros ${EEROS_LIBS} ${ROS_LIBRARIES}  ${MSG_LIBRARIES})
  list(APPEND targets rosTest2)

  add_executable(eerosRosTest eerosRosTest.cpp)
  target_link_libraries(eerosRosTest eeros ${EEROS_LIBS} ${ROS_LIBRARIES}  ${MSG_LIBRARIES})
  list(APPEND targets eerosRosTest)

  if(INSTALL_EXAMPLES)
    install(TARGETS ${targets} RUNTIME DESTINATION examples/ros)
    install(FILES ${files} DESTINATION examples/ros)
  endif()
endif()
