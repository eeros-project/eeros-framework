cmake_minimum_required(VERSION 3.14)

include(cmake/git.cmake)

version_from_git(GIT_VERSION)
message("git version: ${GIT_VERSION}")

project(EEROS VERSION ${GIT_VERSION})

include(GNUInstallDirs)
include(CMakeDependentOption)

include(cmake/package_management.cmake)

include(cmake/platform.cmake)

include(CMakePackageConfigHelpers)

# # Add subdirectories
add_subdirectory(src) # EEROS framework

add_library(${PROJECT_NAME}_eeros SHARED ${EEROS_SRCS})
add_library(${PROJECT_NAME}::eeros ALIAS ${PROJECT_NAME}_eeros)
set_target_properties(${PROJECT_NAME}_eeros PROPERTIES
  OUTPUT_NAME eeros
  EXPORT_NAME eeros)

# include the eeros headers + eeros/config.cpp generated earlier
# There is probably a better way to do this (config.hpp) and ideally the headers should be part of the project sources anyway
target_include_directories(${PROJECT_NAME}_eeros
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/includes>
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>

  $<INSTALL_INTERFACE:include>)

set_target_properties(${PROJECT_NAME}_eeros PROPERTIES VERSION ${EEROS_VERSION})

target_compile_options(${PROJECT_NAME}_eeros PRIVATE -g -Wall)
target_compile_features(${PROJECT_NAME}_eeros PUBLIC cxx_std_20)

find_package(Doxygen)
eeros_find_package(${PROJECT_NAME}_eeros Threads)
find_file(LIBCURSES "curses.h" ${ADDITIONAL_INCLUDE_DIRS})

option(LIB_ONLY_BUILD "Only build library, no tools or examples" FALSE)
cmake_dependent_option(BUILD_EXAMLES "Also build examples" TRUE "NOT LIB_ONLY_BUILD" FALSE)
cmake_dependent_option(BUILD_TOOLS "Also build examples" TRUE "NOT LIB_ONLY_BUILD" FALSE)
cmake_dependent_option(USE_TESTS "Also build tests" FALSE "NOT LIB_ONLY_BUILD" FALSE)

option(BUILD_LIBUCL "Build libucl rather than relying on the system to provide it" ON)

if(BUILD_LIBUCL)
  include(cmake/libucl.cmake)
endif()

if(NOT BUILD_TOOLS)
  add_subdirectory(tools)
endif()

if(BUILD_EXAMLES)
  add_subdirectory(examples)
endif()

target_link_libraries(${PROJECT_NAME}_eeros PUBLIC Threads::Threads ucl PRIVATE rt ${CMAKE_DL_LIBS})

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
set(EEROS_DOC_OUTDIR "./doc" CACHE PATH "Output directory for EEROS docs")
if(DOXYGEN_FOUND)
  configure_file(doc/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  add_custom_target(${PROJECT_NAME}_docs ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif(DOXYGEN_FOUND)

# # Use ncurses library if available
if(LIBCURSES)
  message(STATUS "-> libcurses available")
  target_link_libraries(${PROJECT_NAME}_eeros PRIVATE ncurses)
  message(STATUS "")
endif()

# # Enable CTest
enable_testing()

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

install(TARGETS ${PROJECT_NAME}_eeros
  EXPORT EEROS
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(EXPORT EEROS NAMESPACE ${PROJECT_NAME}:: DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
get_target_property(EEROS_CMAKE_PACKAGE_DEPENDS ${PROJECT_NAME}_eeros EEROS_CMAKE_PACKAGE_DEPENDS)
configure_file(cmake/EEROSConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/EEROSConfig.cmake @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/EEROSConfig.cmake DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

install(
  FILES
  ${CMAKE_CURRENT_BINARY_DIR}/EEROSConfigVersion.cmake
  DESTINATION
  ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

# # Set special variable for eclipse IDE
set(CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT ON)
