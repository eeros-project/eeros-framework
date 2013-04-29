if(UNIX AND NOT APPLE)
add_eeros_sources(ExecutorService_Linux.cpp)
elseif(WIN32)
add_eeros_sources(ExecutorService_Windows.cpp)
endif()

add_eeros_sources(Executor.cpp)