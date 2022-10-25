add_executable(loggerTest LoggerTest.cpp)
target_link_libraries(loggerTest eeros ${EEROS_LIBS} ${ROS_LIBRARIES})

if(INSTALL_EXAMPLES)
  install(TARGETS loggerTest RUNTIME DESTINATION examples/logger)
endif()

