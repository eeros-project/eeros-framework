add_executable(canExample1 CANexample1.cpp)
target_link_libraries(canExample1 eeros ${EEROS_LIBS} ${EXTERNAL_LIBS})

if(INSTALL_EXAMPLES)
  install(TARGETS canExample1 RUNTIME DESTINATION examples/can)
endif()

if (canopen_FOUND)
  add_executable(canExample2 CANexample2.cpp)
	target_link_libraries(canExample2 eeros ${EEROS_LIBS} ${EXTERNAL_LIBS})

  if(INSTALL_EXAMPLES)
    install(TARGETS canExample2 RUNTIME DESTINATION examples/can)
  endif()
endif()

