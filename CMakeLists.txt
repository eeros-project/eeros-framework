cmake_minimum_required(VERSION 2.8)

project(EEROS)

add_subdirectory(src)
add_subdirectory(test)

#SET(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(${EEROS_PUBLIC_INCLUDE_DIR})
#SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/build/lib)
#SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/build/bin)

#add_executable(test Test.cpp)
#target_link_libraries(test complex)

SET(CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT ON)

