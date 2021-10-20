add_eeros_sources(HAL.cpp JsonParser.cpp)

if(LINUX)
	add_eeros_sources(SysFsDigIn.cpp SysFsDigOut.cpp XBox.cpp Mouse.cpp Keyboard.cpp SpaceNavigator.cpp ) 
	
	#if(USE_ODRIVE)
		add_eeros_sources(ODrive_USB.cpp ODrive_UART.cpp)
	#endif()
	
	#if(USE_SBGELLIPSE)
		add_eeros_sources(SbgEllipseA.cpp)
	#endif()
	
	#if(USE_REALSENSE)
# 		add_eeros_sources(RealsenseT265.cpp)
	#endif()
	
	#if(USE_BAUMER)
		add_eeros_sources(BaumerOM70.cpp)
	#endif()
	
	#if(USE_RPLIDAR)
		add_eeros_sources(RPLidar.cpp)
	#endif()
endif()
