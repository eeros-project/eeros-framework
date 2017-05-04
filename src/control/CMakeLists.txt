
add_eeros_sources(Block.cpp TimeDomain.cpp TransitionBlock.cpp Vector2Corrector.cpp Signal.cpp PathPlanner.cpp NotConnectedFault.cpp NaNOutputFault.cpp)

if(LINUX)
	add_eeros_sources(XBoxInput.cpp MouseInput.cpp KeyboardInput.cpp)
endif()
