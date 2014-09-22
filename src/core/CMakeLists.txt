# Platform specific source files
if(LINUX)
	add_eeros_sources(System_Linux.cpp PeriodicThread_Linux.cpp SharedMemory.cpp)
elseif(WINDOWS)
	add_eeros_sources(System_Windows.cpp PeriodicThread_Windows.cpp)
endif()

# Platform independent source files 
add_eeros_sources(Thread.cpp EEROSException.cpp)
