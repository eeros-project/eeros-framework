
##### UNIT TESTS FOR MATH #####

add_subdirectory(matrix)
add_subdirectory(frames)

set(EEROS_TEST_SRCS ${EEROS_TEST_SRCS} PARENT_SCOPE)	# force the propagation of the test sources to the parent dir
