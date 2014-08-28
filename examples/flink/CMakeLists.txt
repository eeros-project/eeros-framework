# compile this example only if flinklib is available...
if(LIBFLINK)
	add_executable(flinkdemo flinkdemo.cpp ControlSystem.cpp)
	target_link_libraries(flinkdemo eeros flink)
endif()
