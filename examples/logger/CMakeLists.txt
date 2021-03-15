add_executable(loggerTest LoggerTest.cpp)
target_link_libraries(loggerTest eeros ${EEROS_LIBS})

if(INSTALL_EXAMPLES)
  install(TARGETS loggerTest RUNTIME DESTINATION examples/logger)
endif()

