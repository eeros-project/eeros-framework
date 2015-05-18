## Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
endif()

add_executable(safetySystemExample ExampleSafetyProperties.cpp safetySystemExample.cpp)
target_link_libraries(safetySystemExample eeros pthread)
