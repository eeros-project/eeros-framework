include(cmake/package_management.cmake)

# use $ROS_DISTRO as a simple way to detect if ROS was sourced
# makes for a much nicer error message for an easy to make mistake
if(DEFINED ENV{ROS_DISTRO})
    target_compile_definitions(eeros PUBLIC USE_ROS2)

    eeros_find_package(eeros ament_cmake)
    eeros_find_package(eeros rclcpp)
    eeros_find_package(eeros std_msgs)
    eeros_find_package(eeros example_interfaces)
    eeros_find_package(eeros sensor_msgs)
    eeros_find_package(eeros nav_msgs)
    eeros_find_package(eeros tf2)

    # for some reason eeros_find_package on its own doesn't work
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(example_interfaces REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(tf2 REQUIRED)
    ament_target_dependencies(eeros PUBLIC rclcpp std_msgs example_interfaces sensor_msgs nav_msgs tf2)
else()
    message(FATAL_ERROR "ROS_DISTRO is not defined! Did you source ROS2 setup.bash?")
endif()
