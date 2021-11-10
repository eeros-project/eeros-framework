cmake_minimum_required(VERSION 2.8.3)
project(odrive_cpp_sdk)

add_compile_options(-std=c++11)


find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)
include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})

include_directories(include include)

add_library(odrive src/odrive.cpp src/odriveEP.cpp)
target_link_libraries(odrive ${LIBUSB1_LIBRARIES} jsoncpp)

#add_executable(odriveTest src/main.cpp)
#target_link_libraries(odriveTest odrive)
