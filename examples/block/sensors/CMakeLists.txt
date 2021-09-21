# set(CMAKE_PREFIX_PATH pathTo/canopenlib/install/)
# 
# find_package(canopenlib)
# 
# include_directories(${canopenlib_INCLUDE_DIR})
# link_directories(${canopenlib_DIR})
# 
# add_executable(main main.cpp)
# target_link_libraries(main canopenlib)


if (canopenlib_FOUND )
	add_executable(canExample CanExample.cpp)
	target_link_libraries(canExample eeros canopenlib ${EEROS_LIBS})

  if(INSTALL_EXAMPLES)
    install(TARGETS canExample RUNTIME DESTINATION examples/can)
  endif()
endif()

