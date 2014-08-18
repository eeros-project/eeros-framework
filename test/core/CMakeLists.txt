
add_executable(ringbuffer RingBufferTest.cpp)
target_link_libraries(ringbuffer eeros)
add_test(core/ringbuffer ringbuffer)

add_executable(ringbufferProcess RingBufferProcessTest.cpp)
target_link_libraries(ringbufferProcess eeros)
add_test(core/ringbuffer/process ringbufferProcess)

add_executable(threadTest ThreadTest.cpp)
target_link_libraries(threadTest eeros)
add_test(core/thread/base threadTest)

add_executable(systemTimeTest SystemTimeTest.cpp)
target_link_libraries(systemTimeTest eeros)
add_test(core/system/getTime systemTimeTest)
