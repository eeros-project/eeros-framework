# Platform specific source files
if(LINUX)
	add_eeros_sources(SharedMemory.cpp)
	if(XENOMAI)
		add_eeros_sources(ExecutorService_Xenomai.cpp) 
	else()
		add_eeros_sources(ExecutorService_Linux.cpp)
	endif()
elseif(WINDOWS)
	add_eeros_sources(ExecutorService_Windows.cpp)
endif()

# Platform independent source files 
add_eeros_sources(Executor.cpp System.cpp RingBuffer.cpp)