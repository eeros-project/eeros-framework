
add_executable(ringbufferTest RingBufferTest.cpp)
target_link_libraries(ringbufferTest ${PROJECT_NAME}_eeros ${EEROS_LIBS})
add_test(core/ringbuffer ringbufferTest)

add_executable(systemTimeTest SystemTimeTest.cpp)
target_link_libraries(systemTimeTest ${PROJECT_NAME}_eeros ${EEROS_LIBS})
add_test(core/system/getTime systemTimeTest)
