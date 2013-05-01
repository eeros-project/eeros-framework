# Main CMake file for the EEROS Framework
# Created on: 22.04.2013
# Author: Martin Zueger

# We need CMake 2.8 or newer
cmake_minimum_required(VERSION 2.8)

# EEROS Robotic Framework
project(EEROS)

# Load CMake modules
include(CheckIncludeFiles)
include(CheckSymbolExists)
include(CheckLibraryExists)

# Determine the target plattform
if("${CMAKE_TOOLCHAIN_FILE}" STREQUAL "")
	# Compile for this platform -> use informations form running system
	if(WIN32 AND NOT CYGWIN) # Windows (native)
		set(WINDOWS TRUE)
	elseif(UNIX) # Linux, BSD, Mac OS X, ...
		set(POSIX TRUE)
		if(${CMAKE_SYSTEM_NAME} MATCHES "Linux") # Linux
			set(LINUX TRUE) # TODO check for Preempt_RT and Xenomai!
		elseif(${CMAKE_SYSTEM_NAME} MATCHES "FreeBSD") # FreeBSD
			set(FREEBSD TRUE)
		elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # Mac OS X
			set(MACOSX TRUE)
		else()
			 # Unkown UNIX
		endif()
	else()
		 # Unkown OS
	endif()
else()
	# Cross compile -> use informations form CMAKE_TOOLCHAIN_FILE
	# TODO
endif()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/includes/config.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/config.hpp)



# Special settings for different IDEs
SET(CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT ON)

# Macros
macro (add_test_sources)
    file (RELATIVE_PATH _relPath "${CMAKE_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach (_src ${ARGN})
        if(_relPath)
            list (APPEND TEST_SRCS "${_relPath}/${_src}")
        else()
            list (APPEND TEST_SRCS "${_src}")
        endif()
    endforeach()
    if (_relPath)
        # propagate TEST_SRCS to parent directory
        set(TEST_SRCS ${TEST_SRCS} PARENT_SCOPE)
    endif()
endmacro()

# EEROS framework source code
add_subdirectory(src)

# Unit tests source code
add_subdirectory(test)