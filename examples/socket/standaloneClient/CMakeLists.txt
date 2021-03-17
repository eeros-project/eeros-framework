add_executable(standaloneSocketClientExample StandaloneClient.cpp)

if(INSTALL_EXAMPLES)
  install(TARGETS standaloneSocketClientExample RUNTIME DESTINATION examples/socket/standaloneClient)
endif()


