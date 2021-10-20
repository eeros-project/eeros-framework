set(targets "")

# if (modbus_FOUND)
    add_executable(baumer_test BaumerOM70Test.cpp)
    target_link_libraries(baumer_test  modbus eeros ucl ${CMAKE_DL_LIBS} ${ROS_LIBRARIES})
    list(APPEND targets baumer_test)
    
#     add_executable(baumer_test_basic  BaumerOM70TestBasic.cpp)
# 	target_link_libraries(baumer_test_basic modbus eeros ucl ${CMAKE_DL_LIBS} ${ROS_LIBRARIES})
#     list(APPEND targets baumer_test_basic)
# endif()

# if (pixycamlib_FOUND)
#     add_executable(pixycam_test PixyCamTest.cpp)
#     target_link_libraries(pixycam_test eeros ${EEROS_LIBS} libpixy)
#     list(APPEND targets pixycam_test)
# endif()

# if (realsenselib_FOUND)
# 	include_directories(include ${realsense_INCLUDE_DIR})
# 	include(${PROJECT_SOURCE_DIR}/external/librealsense/CMake/opengl_config.cmake)
# 
#     add_executable(realsense_test RealsenseT265Test.cpp)
# 	set_property(TARGET realsense_test PROPERTY CXX_STANDARD 11)
# 	target_link_libraries(realsense_test eeros ucl ${CMAKE_DL_LIBS} ${ROS_LIBRARIES} ${realsense2_LIBRARY} ${DEPENDENCIES})
# 	target_include_directories(realsense_test PRIVATE ${realsense_INCLUDE_DIR})
#     list(APPEND targets realsense_test)
# endif()

# if (rplidarlib_FOUND)
    add_executable(rplidar_test RPLidarTest.cpp)
	target_link_libraries(rplidar_test rplidar_sdk eeros ${EEROS_LIBS} ucl ${CMAKE_DL_LIBS} ${ROS_LIBRARIES} )
    list(APPEND targets rplidar_test)
# endif()

# if (sbgecomlib_FOUND)
    add_executable(sbg_test SBGEllipseATest.cpp)
    target_link_libraries(sbg_test eeros ${EEROS_LIBS} sbgECom)
    list(APPEND targets sbg_test)
# endif()


if(INSTALL_EXAMPLES)
install(TARGETS ${targets} RUNTIME DESTINATION examples/block/sensors)
endif()
