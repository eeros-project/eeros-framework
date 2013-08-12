# Main CMake file for the EEROS Framework

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
			set(LINUX TRUE) # Linux operating system

			# Check for Xenomai
			set(XENOMAI_SEARCH_PATH /usr/xenomai /usr) # set the search path
			find_path(XENOMAI_DIR include/xeno_config.h ${XENOMAI_SEARCH_PATH}) # find xeno-config.h
			if(XENOMAI_DIR) # Xenomai available
				set(XENOMAI_INCLUDE_DIR ${XENOMAI_DIR}/include) 
				set(XENOMAI_INCLUDE_POSIX_DIR ${XENOMAI_DIR}/include/posix)
				find_library(XENOMAI_LIBRARY_NATIVE  native  ${XENOMAI_DIR}/lib ) 
				find_library(XENOMAI_LIBRARY_XENOMAI xenomai ${XENOMAI_DIR}/lib ) 
				find_library(XENOMAI_LIBRARY_PTHREAD_RT pthread_rt rtdm ${XENOMAI_DIR}/lib ) 
				find_library(XENOMAI_LIBRARY_RTDM    rtdm    ${XENOMAI_DIR}/lib ) 
				find_file(XENOMAI_POSIX_WRAPPERS lib/posix.wrappers ${XENOMAI_SEARCH_PATH} ) # find the posix wrappers 
				set( XENOMAI_EXE_LINKER_FLAGS "-Wl,@${XENOMAI_POSIX_WRAPPERS}" ) # set the linker flags 
				set(XENOMAI_DEFINITIONS "-D_GNU_SOURCE -D_REENTRANT -Wall -pipe -D__XENO__") # add compile/preprocess options 
				include_directories(${XENOMAI_INCLUDE_DIR})
				LINK_LIBRARIES(native ${XENOMAI_LIBRARY_NATIVE})
				LINK_LIBRARIES(xenomai ${XENOMAI_LIBRARY_XENOMAI})
				LINK_LIBRARIES(pthread_rt ${XENOMAI_LIBRARY_PTHREAD_RT})
				LINK_LIBRARIES(rtdm ${XENOMAI_LIBRARY_RTDM})
				set(XENOMAI TRUE)
			endif(XENOMAI_DIR)

			# Check for Preempt_RT
			# TODO
			# set(PREEMPT_RT TRUE)
			
		elseif(${CMAKE_SYSTEM_NAME} MATCHES "FreeBSD") # FreeBSD
			set(FREEBSD TRUE)
		elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # Mac OS X
			set(MACOSX TRUE)
		else()
			 # Unkown UNIX
			 # TODO
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

# Unit tests source code
add_subdirectory(examples)