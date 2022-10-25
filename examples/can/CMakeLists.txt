if (canopenlib_FOUND)
	add_executable(canExample CanExample.cpp)
	target_link_libraries(canExample eeros canopenlib ${EEROS_LIBS} ${ROS_LIBRARIES})

  if(INSTALL_EXAMPLES)
    install(TARGETS canExample RUNTIME DESTINATION examples/can)
  endif()
endif()

