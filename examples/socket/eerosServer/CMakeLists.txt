add_executable(socketServerExample SocketServerExample.cpp)
target_link_libraries(socketServerExample eeros ${CMAKE_DL_LIBS} ucl)

if(INSTALL_EXAMPLES)
  install(TARGETS socketServerExample RUNTIME DESTINATION examples/socket/eerosServer)
endif()

