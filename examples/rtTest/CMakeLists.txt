add_executable(rtTest rtTest.cpp)
target_link_libraries(rtTest eeros ${EEROS_LIBS})

if(INSTALL_EXAMPLES)
  install(TARGETS rtTest RUNTIME DESTINATION examples/rtTest)
endif()

