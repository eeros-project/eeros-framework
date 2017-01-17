add_eeros_sources(HAL.cpp DummyLogicInput.cpp DummyLogicOutput.cpp DummyRealInput.cpp DummyRealOutput.cpp JsonParser.cpp)

if(LINUX)
	add_eeros_sources(SysFsDigIn.cpp SysFsDigOut.cpp Joystick.cpp Mouse.cpp)
endif()
