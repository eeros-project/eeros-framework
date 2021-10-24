add_eeros_sources(HAL.cpp JsonParser.cpp)

if(LINUX)
	add_eeros_sources(SysFsDigIn.cpp SysFsDigOut.cpp XBox.cpp Mouse.cpp Keyboard.cpp SpaceNavigator.cpp) 
	IF(USE_MODBUS)
		add_eeros_sources(BaumerOM70.cpp)
	endif()
endif()
