cmake_minimum_required(VERSION 2.8)

project(EEROS)

#SET(CMAKE_INCLUDE_CURRENT_DIR ON)
#include_directories(${EEROS_PUBLIC_INCLUDE_DIR})

#SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/build/lib)
#SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/build/bin)

SET(CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT ON)



macro (add_test_sources)
    file (RELATIVE_PATH _relPath "${CMAKE_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach (_src ${ARGN})
        if (_relPath)
            list (APPEND TEST_SRCS "${_relPath}/${_src}")
        else()
            list (APPEND TEST_SRCS "${_src}")
        endif()
    endforeach()
    if (_relPath)
        # propagate TEST_SRCS to parent directory
        set (TEST_SRCS ${TEST_SRCS} PARENT_SCOPE)
    endif()
endmacro()

add_subdirectory(src)
add_subdirectory(test)