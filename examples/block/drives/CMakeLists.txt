# Test Basic Odrive functions USB + EEROS

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)
include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})

add_executable(odrive_test_usb  ODriveUSB_Test.cpp )
target_link_libraries(odrive_test_usb eeros ucl odrive ${CMAKE_DL_LIBS} ${ROS_LIBRARIES} ${LIBUSB1_LIBRARIES} jsoncpp)

