
add_eeros_sources(
    Block.cpp 
    TimeDomain.cpp 
    Vector2Corrector.cpp 
    Signal.cpp 
    NotConnectedFault.cpp 
    NaNOutputFault.cpp
    IndexOutOfBoundsFault.cpp
#     sensors/RPLidarInput.cpp
#     sensors/RealsenseT265Input.cpp
#    sensors/SBGEllipseAInput.cpp 
#     sensors/BaumerOM70Input.cpp
#     filter/Filter.cpp
    )

if(LINUX)
	add_eeros_sources(XBoxInput.cpp MouseInput.cpp SpaceNavigatorInput.cpp)
endif()
