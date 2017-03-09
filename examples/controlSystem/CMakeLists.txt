add_executable(periodicExample1 PeriodicExample1.cpp)
target_link_libraries(periodicExample1 eeros pthread)

add_executable(periodicExample2 PeriodicExample2.cpp)
target_link_libraries(periodicExample2 eeros pthread)

add_executable(blockTest1 BlockTest1.cpp)
target_link_libraries(blockTest1 eeros pthread)

add_executable(blockTest2 BlockTest2.cpp)
target_link_libraries(blockTest2 eeros pthread)
