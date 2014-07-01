# Main CMake file for the EEROS Framework

# We need CMake 2.8 or newer
cmake_minimum_required(VERSION 2.8)

# EEROS Robotic Framework
project(EEROS)

# Additional include and link directories
include_directories(${ADDITIONAL_INCLUDE_DIRS})
link_directories(${ADDITIONAL_LINK_DIRS})

# Install directive for the header files
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/includes/eeros DESTINATION include)

# Load CMake modules
include(CheckIncludeFiles)
include(CheckSymbolExists)
include(CheckLibraryExists)
include(CheckCXXCompilerFlag)

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
				set(REALTIME_SUPPORT TRUE)
			endif(XENOMAI_DIR)

			# Check for Preempt_RT
			# TODO
			set(PREEMPT_RT FALSE)
			
			# Check for libraries
			if(NOT LIBFLINK)
				CHECK_INCLUDE_FILES ("flinklib.h" LIBFLINK)
			endif()
			if(NOT LIBCOMEDI)
				CHECK_INCLUDE_FILES ("comedilib.h" LIBCOMEDI)
			endif()

			if(LIBCOMEDI)
				SET(EXTERNAL_LIBS ${EXTERNAL_LIBS} comedi)
			endif()
			if(LIBFLINK)
				SET(EXTERNAL_LIBS ${EXTERNAL_LIBS} flink)
			endif()
			
		elseif(${CMAKE_SYSTEM_NAME} MATCHES "FreeBSD") # FreeBSD
			set(FREEBSD TRUE)
		elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # Mac OS X
			set(MACOSX TRUE)
		else()
			# Unkown UNIX
			# nothing to do
		endif()
	else()
		 # Unkown OS
		 # nothing to do
	endif()
else()
	# Cross compile -> use informations form CMAKE_TOOLCHAIN_FILE
	# nothing to do
endif()

## Check and set C++ compiler flags
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler!")
endif()

## Config include file
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/includes/config.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/eeros/config.hpp)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/eeros/config.hpp DESTINATION include/eeros)

# Special settings for different IDEs
SET(CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT ON)

# EEROS framework
add_subdirectory(src)

# Unit tests
enable_testing()
add_subdirectory(test)

# Examples
add_subdirectory(examples)
