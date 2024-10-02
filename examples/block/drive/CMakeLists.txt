include(${PROJECT_SOURCE_DIR}/cmake/target_management.cmake)
eeros_add_dependent_target(odriveUSBTest DEPENDS_ON USE_ODRIVE SOURCES ODriveUSBTest.cpp)