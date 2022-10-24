cmake_minimum_required(VERSION 3.10)

project(EEROS)

include(GNUInstallDirs)

## Load CMake modules library at specified version
set(LOAD_CMAKE_MODULES_LIB TRUE CACHE BOOL "TRUE if the cmake-modules library should be loaded.")

if(LOAD_CMAKE_MODULES_LIB)
  include(cmake/CMakeModulesLibLoader.cmake)
  load_cmake_modules_lib(https://github.com/eeros-project/cmake-modules.git)
endif()
include(CMakeModulesLib)
if(LOAD_CMAKE_MODULES_LIB)
  checkout_cmake_modules_lib_version(a50add2)
endif()

if(CMAKE_BUILD_TYPE MATCHES "Debug|DEBUG")
  logger_on()
endif()

## Fetch the version information from git tag
include(VersioningHelpers)
setLibraryVersion()

set(EEROS_VERSION_MAJOR ${VERSION_MAJOR})
set(EEROS_VERSION_MINOR ${VERSION_MINOR})
set(EEROS_VERSION_PATCH ${VERSION_PATCH})
set(EEROS_VERSION_TWEAK ${VERSION_TWEAK})
set(EEROS_VERSION ${EEROS_VERSION_MAJOR}.${EEROS_VERSION_MINOR}.${EEROS_VERSION_PATCH}.${EEROS_VERSION_TWEAK})

log("Going to build EEROS v${EEROS_VERSION}")

include(CMakePackageConfigHelpers)


## Check environment
if(WIN32 AND NOT CYGWIN)
  set(WINDOWS TRUE) # Windows (native)
elseif(UNIX)
  set(POSIX TRUE) # Linux, BSD, Mac OS X, ...
  if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(LINUX TRUE) # GNU/Linux
  else()
    # Unkown UNIX
    message(WARNING "Unknown UNIX operating system!")
  endif()
else()
  # Unkown OS
  message(WARNING "Unknown operating system!")
endif()

## download and builds libucl
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

# Add libucl directly to our build. This defines the libucl target.
add_subdirectory(${EEROS_BINARY_DIR}/libucl-src
                ${EEROS_BINARY_DIR}/libucl-build)

## Add subdirectories
add_subdirectory(src) # EEROS framework

add_library(eeros SHARED ${EEROS_SRCS})

# include the eeros headers + eeros/config.cpp generateed earlier
# There is probably a better way to do this (config.hpp) and ideally the headers should be part of the project sources anyway
target_include_directories(eeros
                            PUBLIC 
                              $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/includes> 
                              $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>

                              $<INSTALL_INTERFACE:include>)

                              
set_target_properties(eeros PROPERTIES
                        VERSION ${EEROS_VERSION})

target_compile_options(eeros PRIVATE -g -Wall PUBLIC -pthread)
target_compile_features(eeros PUBLIC cxx_std_17)

target_link_libraries(eeros PUBLIC pthread PRIVATE ucl ${CMAKE_DL_LIBS})

if(LINUX)
  target_link_libraries(eeros PRIVATE rt)
endif()

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/EEROSConfigVersion.cmake
  VERSION ${EEROS_VERSION}
  COMPATIBILITY SameMinorVersion)

## Configure and install config header file
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/includes/config.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/eeros/config.hpp)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/eeros/config.hpp DESTINATION include/eeros)

## Install all eeros header files
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/includes/eeros DESTINATION include)

install(TARGETS eeros
        EXPORT EEROS
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(EXPORT EEROS DESTINATION ${CMAKE_INSTALL_LIBDIR}/EEROS/cmake)
install(FILES EEROSConfig.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/EEROS/cmake)

install(
  FILES
    # ${CMAKE_CURRENT_BINARY_DIR}/EEROSConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/EEROSConfigVersion.cmake
  DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/EEROS/cmake)


## Use ncurses library if available
find_file(LIBCURSES "curses.h" ${ADDITIONAL_INCLUDE_DIRS})
if(LIBCURSES)
  message(STATUS "-> libcurses available")
  target_link_libraries(eeros PRIVATE ncurses)

endif()

## Use Find package ecmasterlib (EtherCAT) if found
message(STATUS "--")
message(STATUS "**********************************")
message(STATUS "EtherCAT")
message(STATUS "**********************************")
find_package(ecmasterlib QUIET)

if (USE_ETHERCAT)
  message(STATUS "--> EtherCAT will be used")
  find_package(ecmasterlib REQUIRED)
  target_link_libraries(eeros PUBLIC ecmasterlib)
  target_compile_definitions(eeros PUBLIC USE_ETHERCAT EEROS_ETHERCAT)
endif()


# TODO: update for target based configuration

## Use CAN if instructed
if(USE_CAN)
  message(STATUS "**********************************")
  message(STATUS "Use canopenlib")
  message(STATUS "**********************************")
  find_package(canopenlib QUIET)
  include_directories(${canopenlib_INCLUDE_DIR})
  link_directories(${canopenlib_DIR})
  if (canopenlib_FOUND)
    message(STATUS "--> 'canopenlib' is found, CANOpen examples will be built.")
  else()
    message(STATUS "--> 'canopenlib' is not installed. CANOpen will NOT be used.")
  endif()
  message(STATUS "")
endif()

## Use MODBUS if instructed
if(USE_MODBUS)
  message(STATUS "**********************************")
  message(STATUS "Use modbus")
  message(STATUS "**********************************")
  list(APPEND EXTERNAL_LIBS modbus)
  message(STATUS "")
endif()



## Use PIXYCAM if instructed
if(USE_PIXYCAM)
  message(STATUS "**********************************")
  message(STATUS "Use pixy cam")
  message(STATUS "**********************************")
  list(APPEND EXTERNAL_LIBS pixy2 usb-1.0)
  message(STATUS "")
endif()



## Use REALSENSE if instructed
if(USE_REALSENSE)
  message(STATUS "**********************************")
  message(STATUS "Use realsense cam")
  message(STATUS "**********************************")
  list(APPEND EXTERNAL_LIBS realsense2)
  message(STATUS "")
endif()



## Use RPLIDAR if instructed
if(USE_RPLIDAR)
  message(STATUS "**********************************")
  message(STATUS "Use rp lidar")
  message(STATUS "**********************************")
  list(APPEND EXTERNAL_LIBS sl_lidar_sdk)
  message(STATUS "")
endif()



## Use SBGECOMM if instructed
if(USE_SBGECOMM)
  message(STATUS "**********************************")
  message(STATUS "Use ellipse sensors")
  message(STATUS "**********************************")
  list(APPEND EXTERNAL_LIBS sbgECom)
  message(STATUS "")
endif()



## Use ODRIVE if instructed
if(USE_ODRIVE)
  message(STATUS "**********************************")
  message(STATUS "Use ODrive")
  message(STATUS "**********************************")
  list(APPEND EXTERNAL_LIBS odrive jsoncpp)
  message(STATUS "")
endif()



## Use ROS if found and instructed
message(STATUS "ROS")
message(STATUS "**********************************")
message( STATUS "ROS_ROOT: " $ENV{ROS_ROOT} )
find_package( roslib )

if (roslib_FOUND AND USE_ROS)
  message( STATUS "--> ROS will be used")
#    set( ROS_FOUND true)
#  include_directories( "${roslib_INCLUDE_DIRS}" )
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



## Set special variable for eclipse IDE
set(CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT ON)



## Configure Doxygen documentation target (make doc)
find_package(Doxygen)
if(DOXYGEN_FOUND)
  configure_file(doc/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM )
endif(DOXYGEN_FOUND)



## Set EEROS libraries
#set(EEROS_LIBS ${CMAKE_DL_LIBS} ucl pthread)
set(EEROS_LIBS ucl)

## Enable CTest
enable_testing()

if(NOT LIB_ONLY_BUILD)
  add_subdirectory(tools)
  add_subdirectory(examples)
endif(NOT LIB_ONLY_BUILD)

if(USE_TESTS)
  add_subdirectory(test)
endif(USE_TESTS)
