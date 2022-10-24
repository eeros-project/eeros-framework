# Platform specific source files
if(POSIX)
	add_eeros_sources(System_POSIX.cpp SharedMemory.cpp)
elseif(WINDOWS)
	add_eeros_sources(System_Windows.cpp PeriodicThread_Windows.cpp)
endif()

find_package(rclcpp REQUIRED)
include_directories(${rclcpp_INCLUDE_DIRS})

# Platform independent source files 
add_eeros_sources(
	Version.cpp
	Runnable.cpp
	Thread.cpp
	Fault.cpp
	PeriodicCounter.cpp
	Statistics.cpp
	Semaphore.cpp
	Executor.cpp
)
