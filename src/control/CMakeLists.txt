
add_eeros_sources(Block.cpp TimeDomain.cpp)

if(LINUX)
	add_eeros_sources(XBoxInput.cpp)
endif()
