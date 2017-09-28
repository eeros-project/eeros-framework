add_executable(socketClientExample SocketClientExample.cpp)
target_link_libraries(socketClientExample eeros ${CMAKE_DL_LIBS} ucl)
