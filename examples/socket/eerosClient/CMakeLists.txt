add_executable(socketClientExample SocketClientExample.cpp)
target_link_libraries(socketClientExample eeros ${CMAKE_DL_LIBS} ucl)

if(INSTALL_EXAMPLES)
  install(TARGETS socketClientExample RUNTIME DESTINATION examples/socket/eerosClient)
endif()

