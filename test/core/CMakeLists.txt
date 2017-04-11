
add_executable(ringbufferTest RingBufferTest.cpp)
target_link_libraries(ringbufferTest eeros ucl)
add_test(core/ringbuffer ringbufferTest)

add_executable(config config.cpp)
target_link_libraries(config eeros ucl)
add_test(core/config config)

add_executable(systemTimeTest SystemTimeTest.cpp)
target_link_libraries(systemTimeTest eeros ucl)
add_test(core/system/getTime systemTimeTest)
