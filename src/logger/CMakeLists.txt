# Platform independent source files 
add_eeros_sources(Logger.cpp LogWriter.cpp StreamLogWriter.cpp)

if(UNIX)
	add_eeros_sources(SysLogWriter.cpp)
endif()

if (roslib_FOUND AND USE_ROS)
	add_eeros_sources(ROSLogWriter.cpp)
endif()
