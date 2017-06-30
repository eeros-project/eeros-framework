add_executable(socketServerExample socketServerExample.cpp)
target_link_libraries(socketServerExample eeros ${CMAKE_DL_LIBS} ucl)
