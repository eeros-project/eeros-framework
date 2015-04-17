
add_eeros_sources(Block.cpp TimeDomain.cpp TransitionBlock.cpp Vector2Corrector.cpp)

if(LINUX)
	add_eeros_sources(XBoxInput.cpp MouseInput.cpp)
endif()
