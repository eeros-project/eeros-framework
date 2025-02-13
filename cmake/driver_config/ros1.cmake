# # Use ROS if found and instructed
message(STATUS "**********************************")
message(STATUS "ROS")
message(STATUS "**********************************")
find_package(roslib QUIET)

if(roslib_FOUND)
    message(STATUS "--> 'ROS' is found, ROS examples will be built.")
else()
    message(FATAL_ERROR "ERROR: ROS is not installed or setup.bash (for ROS) was not executed.")
endif()

include_directories("${roslib_INCLUDE_DIRS}")
list(APPEND ROS_LIBRARIES "${roslib_LIBRARIES}")
find_package(rosconsole QUIET)
list(APPEND ROS_LIBRARIES "${rosconsole_LIBRARIES}")
find_package(roscpp QUIET)
list(APPEND ROS_LIBRARIES "${roscpp_LIBRARIES}")
target_link_libraries(${PROJECT_NAME}_eeros PRIVATE ${roslib_LIBRARIES})
add_definitions(-DUSE_ROS)
