
find_package(rclcpp REQUIRED)

target_compile_definitions(eeros PUBLIC USE_ROS2)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
ament_target_dependencies(eeros PUBLIC rclcpp)