
add_executable(ringbufferTest RingBufferTest.cpp)
target_link_libraries(ringbufferTest eeros ${EEROS_LIBS})
add_test(core/ringbuffer ringbufferTest)

add_executable(config config.cpp)
target_link_libraries(config eeros ${EEROS_LIBS})
add_test(core/config config)

add_executable(systemTimeTest SystemTimeTest.cpp)
target_link_libraries(systemTimeTest eeros ${EEROS_LIBS})
add_test(core/system/getTime systemTimeTest)
