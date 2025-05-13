
##### UNIT TESTS FOR MATH #####

add_subdirectory(matrix)
add_subdirectory(frames)
add_subdirectory(tf)

set(EEROS_TEST_SRCS ${EEROS_TEST_SRCS} PARENT_SCOPE)	# force the propagation of the test sources to the parent dir

add_eeros_test_sources(Quaternion.cpp)
