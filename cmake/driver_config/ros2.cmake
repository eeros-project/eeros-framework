# use $ROS_DISTRO as a simple way to detect if ROS was sourced
# makes for a much nicer error message for an easy to make mistake
if(DEFINED ENV{ROS_DISTRO})
    find_package(rclcpp REQUIRED)

    target_compile_definitions(eeros PUBLIC USE_ROS2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    ament_target_dependencies(eeros PUBLIC rclcpp)
else()
    message(FATAL_ERROR "ROS_DISTRO is not defined! Did you source ROS2 setup.bash?")
endif()
