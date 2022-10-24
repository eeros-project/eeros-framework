# Platform independent source files 
add_eeros_sources(Logger.cpp LogWriter.cpp StreamLogWriter.cpp)

find_package(rclcpp REQUIRED)
include_directories(${rclcpp_INCLUDE_DIRS})

if(UNIX)
	add_eeros_sources(SysLogWriter.cpp)
endif()

if(ROS_FOUND)
	add_eeros_sources(ROSLogWriter.cpp)
endif()
