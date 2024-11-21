include(cmake/package_management.cmake)

# use $ROS_DISTRO as a simple way to detect if ROS was sourced
# makes for a much nicer error message for an easy to make mistake
if(DEFINED ENV{ROS_DISTRO})
    target_compile_definitions(${PROJECT_NAME}_eeros PUBLIC USE_ROS2)

    eeros_find_package(${PROJECT_NAME}_eeros ament_cmake)
    eeros_find_package(${PROJECT_NAME}_eeros rclcpp)
    eeros_find_package(${PROJECT_NAME}_eeros std_msgs)
    eeros_find_package(${PROJECT_NAME}_eeros example_interfaces)
    eeros_find_package(${PROJECT_NAME}_eeros sensor_msgs)
    eeros_find_package(${PROJECT_NAME}_eeros nav_msgs)
    eeros_find_package(${PROJECT_NAME}_eeros tf2)
    eeros_add_package(${PROJECT_NAME}_eeros eeros_msgs)

    # for some reason eeros_find_package on its own doesn't work
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(example_interfaces REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(tf2 REQUIRED)
    ament_target_dependencies(${PROJECT_NAME}_eeros PUBLIC rclcpp std_msgs example_interfaces sensor_msgs nav_msgs tf2)

    add_subdirectory(eeros_msgs)
    target_link_libraries(${PROJECT_NAME}_eeros PUBLIC eeros_msgs__rosidl_typesupport_cpp)

else()
    message(FATAL_ERROR "ROS_DISTRO is not defined! Did you source ROS2 setup.bash?")
endif()
