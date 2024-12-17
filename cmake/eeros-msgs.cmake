include(FetchContent)

FetchContent_Declare(eeros-msgs
    GIT_REPOSITORY https://github.com/eeros-project/ros-eeros-msgs.git
    GIT_TAG main
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/eeros_msgs"
    BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/eeros_msgs")

FetchContent_MakeAvailable(eeros-msgs)
eeros_add_package(${PROJECT_NAME}_eeros eeros_msgs)
