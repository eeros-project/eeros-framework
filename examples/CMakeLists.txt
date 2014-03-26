include_directories(${EEROS_SOURCE_DIR}/includes ${EEROS_BINARY_DIR})

add_subdirectory(sequencer)

if(LIBCOMEDI)
	add_subdirectory(safety)
	add_subdirectory(simpleMotorController)
endif()
