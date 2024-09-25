include(cmake/package_management.cmake)

# use $ROS_DISTRO as a simple way to detect if ROS was sourced
# makes for a much nicer error message for an easy to make mistake
if(DEFINED ENV{ROS_DISTRO})
    target_compile_definitions(eeros PUBLIC USE_ROS2)

    eeros_find_package(eeros ament_cmake)
    eeros_find_package(eeros rclcpp)

    # for some reason eeros_find_package on its own doesn't work
    find_package(rclcpp REQUIRED)
    ament_target_dependencies(eeros PUBLIC rclcpp)
else()
    message(FATAL_ERROR "ROS_DISTRO is not defined! Did you source ROS2 setup.bash?")
endif()
