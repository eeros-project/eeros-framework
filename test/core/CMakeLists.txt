
add_executable(ringbuffer RingBufferTest.cpp)
target_link_libraries(ringbuffer eeros)
add_test(core/ringbuffer ringbuffer)

add_executable(ringbufferProcess RingBufferProcessTest.cpp)
target_link_libraries(ringbufferProcess eeros)
add_test(core/ringbuffer/process ringbufferProcess)
