
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
endif()

if(USE_ROS2)
  add_subdirectory(ros2)
endif()
