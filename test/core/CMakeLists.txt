
add_executable(ringbufferTest RingBufferTest.cpp)
target_link_libraries(ringbufferTest eeros)
add_test(core/ringbuffer ringbufferTest)

add_executable(config config.cpp)
target_link_libraries(config eeros)
add_test(core/config config)

add_executable(threadTest ThreadTest.cpp)
target_link_libraries(threadTest eeros)
add_test(core/thread/base threadTest)

add_executable(systemTimeTest SystemTimeTest.cpp)
target_link_libraries(systemTimeTest eeros)
add_test(core/system/getTime systemTimeTest)
