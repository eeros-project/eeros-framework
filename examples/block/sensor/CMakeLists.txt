set(targets "")

if (USE_MODBUS)
  add_executable(baumerTest BaumerOM70Test.cpp)
  target_link_libraries(baumerTest eeros ${EEROS_LIBS})
  list(APPEND targets baumerTest)
    
  add_executable(baumerTestBasic  BaumerOM70TestBasic.cpp)
  target_link_libraries(baumerTestBasic)
  list(APPEND targets baumerTestBasic)
endif()

if (USE_PIXYCAM)
  add_executable(pixycamTest PixyCamTest.cpp)
  target_link_libraries(pixycamTest eeros ${EEROS_LIBS})
  list(APPEND targets pixycamTest)
endif()

if (USE_REALSENSE)
  add_executable(realsenseTest RealsenseT265Test.cpp)
  target_link_libraries(realsenseTest eeros ${EEROS_LIBS})
  list(APPEND targets realsenseTest)
endif()

if (USE_RPLIDAR)
  add_executable(rplidarTest RPLidarTest.cpp)
  target_link_libraries(rplidarTest eeros ${EEROS_LIBS})
  list(APPEND targets rplidarTest)
endif()

if (USE_SBGECOMM)
  add_executable(sbgTest SBGEllipseATest.cpp)
  target_link_libraries(sbgTest eeros ${EEROS_LIBS} )
  list(APPEND targets sbgTest)
endif()


if(INSTALL_EXAMPLES)
  install(TARGETS ${targets} RUNTIME DESTINATION examples/block/sensor)
endif()
