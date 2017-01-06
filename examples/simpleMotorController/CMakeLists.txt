# compile this example only if libcomedi is available...
#add_executable(simpleMotorControler SimpleMotorController.cpp MyControlSystem.cpp MySafetyProperties.cpp SequenceA.cpp)
if(LIBCOMEDI)
	target_link_libraries(simpleMotorControler eeros comedi)
endif()
if(LIBFLINK)
	target_link_libraries(simpleMotorControler eeros flink)
endif()