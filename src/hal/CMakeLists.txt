add_eeros_sources(HAL.cpp DummyLogicInput.cpp DummyLogicOutput.cpp DummyRealInput.cpp DummyRealOutput.cpp)

if(LIBCOMEDI)
	add_eeros_sources(ComediDevice.cpp ComediDigIn.cpp ComediDigOut.cpp ComediAdc.cpp ComediDac.cpp ComediFqd.cpp)
endif()

if(LIBFLINK)
	add_eeros_sources(FlinkDevice.cpp FlinkPwm.cpp FlinkFqd.cpp FlinkDigOut.cpp)
endif()
