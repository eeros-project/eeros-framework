cmake_minimum_required(VERSION 3.14)

project(EEROS)

include(GNUInstallDirs)

include(cmake/CMakeModulesLib.cmake)
include(cmake/package_management.cmake)

if(CMAKE_BUILD_TYPE MATCHES "Debug|DEBUG")
  logger_on()
endif()

include(cmake/platform.cmake)

# # Fetch the version information from git tag
include(VersioningHelpers)
setLibraryVersion()

set(EEROS_VERSION_MAJOR ${VERSION_MAJOR})
set(EEROS_VERSION_MINOR ${VERSION_MINOR})
set(EEROS_VERSION_PATCH ${VERSION_PATCH})
set(EEROS_VERSION_TWEAK ${VERSION_TWEAK})
set(EEROS_VERSION ${EEROS_VERSION_MAJOR}.${EEROS_VERSION_MINOR}.${EEROS_VERSION_PATCH}.${EEROS_VERSION_TWEAK})

log("Going to build EEROS v${EEROS_VERSION}")

include(CMakePackageConfigHelpers)

# # Add subdirectories
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

target_compile_options(eeros PRIVATE -g -Wall)
target_compile_features(eeros PUBLIC cxx_std_17)

find_package(Doxygen)
eeros_find_package(eeros Threads)
find_file(LIBCURSES "curses.h" ${ADDITIONAL_INCLUDE_DIRS})
include(cmake/libucl.cmake)

target_link_libraries(eeros PUBLIC Threads::Threads PRIVATE ucl rt ${CMAKE_DL_LIBS})

include(cmake/driver_config.cmake)

add_driver_option(FLAG_NAME "USE_ETHERCAT" HELP_TEXT "Enable EtherCAT support" PACKAGE_DEPENDS ecmasterlib LINK_LIBRARIES_PUBLIC ecmasterlib COMPILE_DEFINITIONS USE_ETHERCAT EEROS_ETHERCAT)
add_driver_option(FLAG_NAME "USE_CAN" HELP_TEXT "Enable CAN support" PACKAGE_DEPENDS canopenlib LINK_LIBRARIES_PUBLIC canopenlib::canopen COMPILE_DEFINITIONS USE_CAN)
add_driver_option(FLAG_NAME "USE_MODBUS" HELP_TEXT "Enable Modbus suppport" PKG_CONFIG libmodbus SOURCES src/hal/BaumerOM70.cpp)
add_driver_option(FLAG_NAME "USE_PIXYCAM" HELP_TEXT "Enable Pixycam suppport" PKG_CONFIG pixy2 usb-1.0)
add_driver_option(FLAG_NAME "USE_REALSENSE" HELP_TEXT "Enable Realsense suppport" PKG_CONFIG realsense2 SOURCES src/hal/RealsenseT265.cpp)
add_driver_option(FLAG_NAME "USE_RPLIDAR" HELP_TEXT "Enable RP LIDAR suppport" PKG_CONFIG sl_lidar_sdk SOURCES src/hal/RPLidar.cpp external/rplidar/RPlidarDriver.cpp)
add_driver_option(FLAG_NAME "USE_SBGECOMM" HELP_TEXT "Enable ellipse sensors suppport" PKG_CONFIG sbgECom SOURCES src/hal/SBGEllipseA.cpp)
add_driver_option(FLAG_NAME "USE_ODRIVE" HELP_TEXT "Enable ODrive suppport" PKG_CONFIG odrive jsoncpp)
add_driver_option(FLAG_NAME "USE_ROS" HELP_TEXT "Enable ROS1 support" CMAKE_FILES ros1.cmake)
add_driver_option(FLAG_NAME "USE_ROS2" HELP_TEXT "Enable ROS2 support" CMAKE_FILES ros2.cmake)

if(USE_ROS AND USE_ROS2)
  message(FATAL_ERROR "ROS1 and ROS2 support are mutually exlusive. Enable either ROS1 or ROS2, but not both")
endif()

# # Configure Doxygen documentation target (make doc)
if(DOXYGEN_FOUND)
  configure_file(doc/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif(DOXYGEN_FOUND)

# # Use ncurses library if available
if(LIBCURSES)
  message(STATUS "-> libcurses available")
  target_link_libraries(eeros PRIVATE ncurses)
  message(STATUS "")
endif()

# # Enable CTest
enable_testing()

if(NOT LIB_ONLY_BUILD)
  add_subdirectory(tools)
  add_subdirectory(examples)
endif(NOT LIB_ONLY_BUILD)

if(USE_TESTS)
  add_subdirectory(test)
endif(USE_TESTS)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/EEROSConfigVersion.cmake
  VERSION ${EEROS_VERSION}
  COMPATIBILITY SameMinorVersion)

# # Configure and install config header file
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/includes/config.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/eeros/config.hpp)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/eeros/config.hpp DESTINATION include/eeros)

# # Install all eeros header files
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/includes/eeros DESTINATION include)

install(TARGETS eeros
  EXPORT EEROS
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(EXPORT EEROS NAMESPACE ${PROJECT_NAME}:: DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
install(FILES cmake/EEROSConfig.cmake DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

install(
  FILES
  ${CMAKE_CURRENT_BINARY_DIR}/EEROSConfigVersion.cmake
  DESTINATION
  ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

# # Set special variable for eclipse IDE
set(CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT ON)
