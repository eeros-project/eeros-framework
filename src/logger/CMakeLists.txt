# Platform independent source files 
add_eeros_sources(LogWriter.cpp StreamLogWriter.cpp)

if(UNIX)
	add_eeros_sources(SysLogWriter.cpp)
endif()
