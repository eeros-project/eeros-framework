#################################################
#### Main CMake file for the EEROS Framework ####
#################################################

## Check cmake version, we need 2.8 or newer
cmake_minimum_required(VERSION 2.8)

## EEROS Robotic Framework
project(EEROS)


## Fetch the version information from git
include(cmake/MunkeiVersionFromGit.cmake)
version_from_git()

set(EEROS_VERSION_MAJOR ${VERSION_MAJOR})
set(EEROS_VERSION_MINOR ${VERSION_MINOR})
set(EEROS_VERSION_PATCH ${VERSION_PATCH})
set(EEROS_VERSION_TWEAK ${VERSION_TWEAK})
set(EEROS_VERSION ${EEROS_VERSION_MAJOR}.${EEROS_VERSION_MINOR}.${EEROS_VERSION_PATCH}.${EEROS_VERSION_TWEAK})


## Additional include and link directories
include_directories(${ADDITIONAL_INCLUDE_DIRS})
link_directories(${ADDITIONAL_LINK_DIRS})

## Install directive for the header files
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/includes/eeros DESTINATION include)

## Install CMake configuration file
set(INCLUDE_INSTALL_DIR "include/")
set(LIBUCL_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/libucl-src/include")
set(LIB_INSTALL_DIR lib/)
set(LIBUCL_LINK_DIR "${CMAKE_CURRENT_BINARY_DIR}/libucl-build/")

include(CMakePackageConfigHelpers)

configure_package_config_file(
	cmake/EEROSConfig.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/EEROSConfig.cmake
	INSTALL_DESTINATION ${LIB_INSTALL_DIR}/EEROS/cmake
	PATH_VARS INCLUDE_INSTALL_DIR LIBUCL_INCLUDE_DIR LIB_INSTALL_DIR LIBUCL_LINK_DIR)

write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/EEROSConfigVersion.cmake
	VERSION ${EEROS_VERSION}
	COMPATIBILITY SameMajorVersion)

install(
	FILES
		${CMAKE_CURRENT_BINARY_DIR}/EEROSConfig.cmake
		${CMAKE_CURRENT_BINARY_DIR}/EEROSConfigVersion.cmake
	DESTINATION
		${LIB_INSTALL_DIR}/EEROS/cmake)


## Load CMake modules
include(CheckCXXCompilerFlag)

# Check environment
if(WIN32 AND NOT CYGWIN)
	set(WINDOWS TRUE) # Windows (native)
elseif(UNIX)
	set(POSIX TRUE) # Linux, BSD, Mac OS X, ...
	if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
		set(LINUX TRUE) # GNU/Linux
		
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
		
	elseif(${CMAKE_SYSTEM_NAME} MATCHES "FreeBSD")
		set(FREEBSD TRUE) # FreeBSD
	
	elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
		set(MACOSX TRUE) # Mac OS X
	
	elseif(${CMAKE_SYSTEM_NAME} MATCHES "QNX")
		set(QNX TRUE) # QNX Neutrino

		set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -ansi -D_POSIX_C_SOURCE=199506")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ansi -D_POSIX_C_SOURCE=199506")

	else()
		# Unkown UNIX
		 message(WARNING "Unknown UNIX operating system!")
	endif()
else()
	 # Unkown OS
	 message(WARNING "Unknown operating system!")
endif()

## Check and set C++ compiler flags
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(SEND_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler!")
endif()
 
 
## Compile with debug symbols
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")


find_file(LIBCURSES "curses.h" ${ADDITIONAL_INCLUDE_DIRS})
if(LIBCURSES)
	message(STATUS "-> libcurses available")
	set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ncurses)
endif()

if(LINUX)
	set(EXTERNAL_LIBS ${EXTERNAL_LIBS} -lrt)
endif()

##########################
# load and add libucl
##########################
# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in ${EEROS_BINARY_DIR}/libucl-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${EEROS_BINARY_DIR}/libucl-download )
if(result)
  message(FATAL_ERROR "CMake step for libucl failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${EEROS_BINARY_DIR}/libucl-download )
if(result)
  message(FATAL_ERROR "Build step for libucl failed: ${result}")
endif()

# Add libucl directly to our build. This defines
# the libucl target.
add_subdirectory(${EEROS_BINARY_DIR}/libucl-src
                 ${EEROS_BINARY_DIR}/libucl-build)

include_directories("${libucl_SOURCE_DIR}/include")
link_directories(${libucl_BINARY_DIR})

###### libucl ######


##########################
# EtherCAT support
##########################
## Find package ecmasterlib
message(STATUS "")
message(STATUS "EtherCAT")
message(STATUS "**********************************")
find_package(ecmasterlib QUIET)
#if (ECMASTERLIB_LIB_DIR)	# check if ecmasterlib is installed
#if (USE_ETHERCAT)	# check if ecmasterlib should be used
if (ECMASTERLIB_LIB_DIR AND USE_ETHERCAT)
	message(STATUS "--> EtherCAT will be used")
	include_directories("${ECMASTERLIB_INCLUDE_DIR}")
	link_directories("${ECMASTERLIB_LIB_DIR}")
	set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ecmasterlib)
	set(LIBECMASTER "${ECMASTERLIB_INCLUDE_DIR}")
	add_definitions(-DUSE_ETHERCAT)
endif (ECMASTERLIB_LIB_DIR AND USE_ETHERCAT)

if (NOT ECMASTERLIB_LIB_DIR AND USE_ETHERCAT)
	message( FATAL_ERROR "ERROR: EtherCAT should be used but 'ecmasterlib' can not be found. Please install 'ecmasterlib'.")
endif (NOT ECMASTERLIB_LIB_DIR AND USE_ETHERCAT)

if (ECMASTERLIB_LIB_DIR AND NOT USE_ETHERCAT)
	message(STATUS "--> 'ecmasterlib' is installed, but EtherCAT will NOT be used. You may want to use the -DUSE_ETHERCAT switch with cmake.")
endif (ECMASTERLIB_LIB_DIR AND NOT USE_ETHERCAT)

if (NOT ECMASTERLIB_LIB_DIR AND NOT USE_ETHERCAT)
	message(STATUS "--> 'ecmasterlib' is NOT  installed. EtherCAT will NOT be used.")
endif (NOT ECMASTERLIB_LIB_DIR AND NOT USE_ETHERCAT)
message(STATUS "")



##########################
# ROS support
##########################
message(STATUS "ROS")
message(STATUS "**********************************")
message( STATUS "ROS_ROOT: " $ENV{ROS_ROOT} )
find_package( roslib QUIET )

if (roslib_FOUND AND USE_ROS)
    message( STATUS "--> ROS will be used")
#    set( ROS_FOUND true)
    include_directories( "${roslib_INCLUDE_DIRS}" )
    list(APPEND ROS_LIBRARIES "${roslib_LIBRARIES}")
    find_package( rosconsole QUIET)
    list(APPEND ROS_LIBRARIES "${rosconsole_LIBRARIES}")
    find_package( roscpp QUIET )
    list(APPEND ROS_LIBRARIES "${roscpp_LIBRARIES}")
    list(APPEND EXTERNAL_LIBS "${ROS_LIBRARIES}")
    add_definitions(-DUSE_ROS)
endif (roslib_FOUND AND USE_ROS)

if (NOT roslib_FOUND AND USE_ROS)
	message( FATAL_ERROR "ERROR: ROS should be used but could not be found. ROS may be not installed or setup.bash (for ROS) was not executed.")
endif (NOT roslib_FOUND AND USE_ROS)

if (roslib_FOUND AND NOT USE_ROS)
	message(STATUS "--> ROS is installed, but ROS will NOT be used. You may want to use the -DUSE_ROS switch with cmake.")
endif (roslib_FOUND AND NOT USE_ROS)

if (NOT roslib_FOUND AND NOT USE_ROS)
	message(STATUS "--> ROS is NOT  installed. ROS will NOT be used.")
endif (NOT roslib_FOUND AND NOT USE_ROS)
message(STATUS "")


## Config include file
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/includes/config.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/eeros/config.hpp)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/eeros/config.hpp DESTINATION include/eeros)

## Special settings for different IDEs
set(CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT ON)


## Doxygen documentation target (make doc)
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(doc/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Generating API documentation with Doxygen" VERBATIM )
endif(DOXYGEN_FOUND)

set(EEROS_LIBS ${CMAKE_DL_LIBS} ucl pthread)

## Enable CTest
enable_testing()

## Source directories
add_subdirectory(src)      # EEROS framework
add_subdirectory(examples) # Examples
add_subdirectory(tools)    # Tools
add_subdirectory(test)     # Unit tests
