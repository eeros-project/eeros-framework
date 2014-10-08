# Platform specific source files
if(POSIX)
	add_eeros_sources(System_POSIX.cpp PeriodicThread_POSIX.cpp SharedMemory.cpp)
elseif(WINDOWS)
	add_eeros_sources(System_Windows.cpp PeriodicThread_Windows.cpp)
endif()

# Platform independent source files 
add_eeros_sources(Version.cpp Thread.cpp EEROSException.cpp)
