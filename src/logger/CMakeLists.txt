# Platform independent source files 
add_eeros_sources(Logger.cpp LogWriter.cpp StreamLogWriter.cpp)

if(UNIX)
	add_eeros_sources(SysLogWriter.cpp)
endif()
