add_executable(multithreading multithreading.cpp)
target_link_libraries(multithreading eeros ${EEROS_LIBS} ${ROS_LIBRARIES})

if(INSTALL_EXAMPLES)
  install(TARGETS multithreading RUNTIME DESTINATION examples/devel)
endif()

