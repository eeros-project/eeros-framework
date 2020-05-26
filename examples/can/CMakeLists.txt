if (canopenlib_FOUND)
	add_executable(canExample CanExample.cpp)
	target_link_libraries(canExample eeros canopenlib ${EEROS_LIBS})
endif()

