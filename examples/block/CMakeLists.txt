include(${PROJECT_SOURCE_DIR}/cmake/target_management.cmake)

eeros_copy_file_post_build(copyPathFile path.txt)

eeros_add_target(blockTest BlockTest.cpp)
eeros_add_target(mouseTest MouseTest.cpp)
eeros_add_target(xBoxTest XBoxTest.cpp)
eeros_add_target(keyboardTest KeyboardTest.cpp)
eeros_add_target(traceTest TraceTest.cpp)
eeros_add_target(spaceNavigatorTest SpaceNavigatorTest.cpp)
eeros_add_target(transitionBlockTest1 TransitionBlockTest.cpp)
eeros_add_target(pathPlannerTest1 PathPlannerTest1.cpp)
eeros_add_target(pathPlannerTest2 PathPlannerTest2.cpp)
eeros_add_target(pathPlannerTest3 PathPlannerTest3.cpp copyPathFile)
eeros_add_target(rateLimiterTest RateLimiterTest.cpp)

add_subdirectory(sensor)
add_subdirectory(drive)
