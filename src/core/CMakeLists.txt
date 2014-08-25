# Platform specific source files
if(LINUX)
	add_eeros_sources(System_Linux.cpp SharedMemory.cpp)
	if(XENOMAI)
		add_eeros_sources(ExecutorService_Xenomai.cpp)
	elseif(PREEMPT_RT)
		add_eeros_sources(ExecutorService_Preempt_RT.cpp)
	else()
		add_eeros_sources(ExecutorService_Linux.cpp)
	endif()
elseif(WINDOWS)
	add_eeros_sources(System_Windows.cpp ExecutorService_Windows.cpp)
endif()

# Platform independent source files 
add_eeros_sources(Thread.cpp PeriodicThread_Linux.cpp ContinualThread.cpp Executor.cpp EEROSException.cpp)
