cmake_minimum_required(VERSION 3.14)

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

include(cmake/libucl.cmake)

## Add subdirectories
add_subdirectory(src) # EEROS framework

add_library(eeros SHARED ${EEROS_SRCS})

# include the eeros headers + eeros/config.cpp generated earlier
# There is probably a better way to do this (config.hpp) and ideally the headers should be part of the project sources anyway
target_include_directories(eeros
                            PUBLIC 
                              $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/includes> 
                              $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>

                              $<INSTALL_INTERFACE:include>)

                              
set_target_properties(eeros PROPERTIES VERSION ${EEROS_VERSION})

target_compile_options(eeros PRIVATE -g -Wall PUBLIC -pthread)
target_compile_features(eeros PUBLIC cxx_std_17)

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

install(EXPORT EEROS NAMESPACE ${PROJECT_NAME}:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/EEROS/cmake)
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
  message(STATUS "")
endif()

include(cmake/driver_config.cmake)

add_driver_option(FLAG_NAME "USE_ETHERCAT" HELP_TEXT "Enable EtherCAT support" PACKAGE_DEPENDS ecmasterlib COMPILE_DEFINITIONS USE_ETHERCAT EEROS_ETHERCAT)
add_driver_option(FLAG_NAME "USE_CAN" HELP_TEXT "Enable CAN support" PACKAGE_DEPENDS canopenlib LINK_LIBRARIES canopenlib::canopen COMPILE_DEFINITIONS USE_CAN)
add_driver_option(FLAG_NAME "USE_MODBUS" HELP_TEXT "Enable Modbus suppport" PACKAGE_DEPENDS modbus)
add_driver_option(FLAG_NAME "USE_PIXYCAM" HELP_TEXT "Enable Pixycam suppport" PACKAGE_DEPENDS pixy2 usb-1.0)
add_driver_option(FLAG_NAME "USE_REALSENSE" HELP_TEXT "Enable Realsense suppport" PACKAGE_DEPENDS realsense2)
add_driver_option(FLAG_NAME "USE_RPLIDAR" HELP_TEXT "Enable RP LIDAR suppport" PACKAGE_DEPENDS sl_lidar_sdk)
add_driver_option(FLAG_NAME "USE_SBGECOMM" HELP_TEXT "Enable ellipse sensors suppport" PACKAGE_DEPENDS sbgECom)
add_driver_option(FLAG_NAME "USE_ODRIVE" HELP_TEXT "Enable ODrive suppport" PACKAGE_DEPENDS odrive jsoncpp)
add_driver_option(FLAG_NAME "USE_ROS" HELP_TEXT "Enable ROS1 support" CMAKE_FILES ros1.cmake)
add_driver_option(FLAG_NAME "USE_ROS2" HELP_TEXT "Enable ROS2 support" CMAKE_FILES ros2.cmake)

if(USE_ROS AND USE_ROS2)
message(FATAL_ERROR "ROS1 and ROS2 support are mutually exlusive. Enable either ROS1 or ROS2, but not both")
endif()

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

target_link_libraries(eeros PUBLIC pthread PRIVATE ucl ${CMAKE_DL_LIBS} ${EXTERNAL_LIBS})

if(LINUX)
  target_link_libraries(eeros PRIVATE rt)
endif()

## Enable CTest
enable_testing()

if(NOT LIB_ONLY_BUILD)
  add_subdirectory(tools)
  add_subdirectory(examples)
endif(NOT LIB_ONLY_BUILD)

if(USE_TESTS)
  add_subdirectory(test)
endif(USE_TESTS)

