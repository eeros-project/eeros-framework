set(files "")

add_custom_command(OUTPUT copyConfig1 POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/system/SystemTest1Config.json" "${EEROS_BINARY_DIR}/examples/system")
list(APPEND files ${EEROS_BINARY_DIR}/examples/system/SystemTest1Config.json)

add_custom_command(OUTPUT copyConfig2 POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/system/SystemTest2Config.json" "${EEROS_BINARY_DIR}/examples/system")
list(APPEND files ${EEROS_BINARY_DIR}/examples/system/SystemTest2Config.json)

set(targets "")

add_executable(systemTest1 SystemTest1.cpp copyConfig1)
target_link_libraries(systemTest1 eeros ${EEROS_LIBS} ${ROS_LIBRARIES})
list(APPEND targets systemTest1)

add_executable(systemTest2 SystemTest2.cpp copyConfig2)
target_link_libraries(systemTest2 eeros ${EEROS_LIBS} ${ROS_LIBRARIES})
list(APPEND targets systemTest2)

add_executable(systemTest3 SystemTest3.cpp)
target_link_libraries(systemTest3 eeros ${EEROS_LIBS} ${ROS_LIBRARIES})
list(APPEND targets systemTest3)

add_executable(subSystemTest SubSystemTest.cpp)
target_link_libraries(subSystemTest eeros ${EEROS_LIBS} ${ROS_LIBRARIES})
list(APPEND targets subSystemTest)

add_executable(mockRobotExample MockRobotExample.cpp)
target_link_libraries(mockRobotExample eeros ${EEROS_LIBS} ${ROS_LIBRARIES})
list(APPEND targets mockRobotExample)

if(INSTALL_EXAMPLES)
  install(TARGETS ${targets} RUNTIME DESTINATION examples/system)
  install(FILES ${files} DESTINATION examples/system)
endif()

