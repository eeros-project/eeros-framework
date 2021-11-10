set(targets "")

if (USE_ODRIVE)
  add_executable(odriveUSBTest  ODriveUSBTest.cpp )
  target_link_libraries(odriveUSBTest eeros ${EEROS_LIBS})
endif()
