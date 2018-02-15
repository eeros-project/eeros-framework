
add_eeros_sources(Block.cpp TimeDomain.cpp Vector2Corrector.cpp Signal.cpp PathPlannerCubic.cpp NotConnectedFault.cpp NaNOutputFault.cpp)

if(LINUX)
	add_eeros_sources(XBoxInput.cpp MouseInput.cpp KeyboardInput.cpp SpaceNavigatorInput.cpp)
endif()
