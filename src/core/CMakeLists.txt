# Platform specific source files
if(LINUX)
add_eeros_sources(ExecutorService_Linux.cpp)
elseif(WINDOWS)
add_eeros_sources(ExecutorService_Windows.cpp)
endif()

# Platform independent source files 
add_eeros_sources(Executor.cpp System.cpp)