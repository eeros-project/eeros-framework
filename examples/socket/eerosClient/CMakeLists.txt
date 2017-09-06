add_executable(socketClientExample socketClientExample.cpp)
target_link_libraries(socketClientExample eeros ${CMAKE_DL_LIBS} ucl)
