if (canopenlib_FOUND)
	add_executable(canExample CanExample.cpp)
	target_link_libraries(canExample eeros ${EEROS_LIBS} ${EXTERNAL_LIBS})

  if(INSTALL_EXAMPLES)
    install(TARGETS canExample RUNTIME DESTINATION examples/can)
  endif()
endif()

