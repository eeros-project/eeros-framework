
add_eeros_sources(
    Block.cpp 
    TimeDomain.cpp 
    Vector2Corrector.cpp 
    Signal.cpp 
    NotConnectedFault.cpp 
    NaNOutputFault.cpp
    IndexOutOfBoundsFault.cpp
    )

if(LINUX)
	add_eeros_sources(XBoxInput.cpp MouseInput.cpp SpaceNavigatorInput.cpp)
	
	#if(USE_SBGELLIPSE)
		add_eeros_sources(sensors/SBGEllipseAInput.cpp)
	#endif()
	
# 	if(USE_REALSENSE)
# 		add_eeros_sources(sensors/RealsenseT265Input.cpp)
	#endif()
	
	#if(USE_BAUMER)
		add_eeros_sources(sensors/BaumerOM70Input.cpp)
	#endif()
	
	#if(USE_RPLIDAR)
		add_eeros_sources(sensors/RPLidarInput.cpp)
	#endif()
endif()
