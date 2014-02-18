# compile this example only if libcomedi is available...
if(LIBCOMEDI)
	add_executable(simpleMotorControler SimpleMotorController.cpp MyControlSystem.cpp MySafetyProperties.cpp SequenceA.cpp)
	target_link_libraries(simpleMotorControler eeros comedi)
endif()