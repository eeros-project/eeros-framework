add_eeros_sources(HAL.cpp JsonParser.cpp)

if(LINUX)
	add_eeros_sources(SysFsDigIn.cpp SysFsDigOut.cpp XBox.cpp Mouse.cpp Keyboard.cpp)
endif()
