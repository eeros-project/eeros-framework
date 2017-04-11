##### UNIT TESTS FOR MUX #####

# Compile and link test application
add_executable(muxTest MuxTest.cpp)
target_link_libraries(muxTest eeros ucl)

# Add tests
add_test(control/mux/matrix/1x1/double/random muxTest 1 1 "${TEST_DATA_DIR}/control/mux/mux-matrix-1-double-random.csv")
add_test(control/mux/matrix/3x1/double/random muxTest 3 1 "${TEST_DATA_DIR}/control/mux/mux-matrix-3-double-random.csv")
add_test(control/mux/matrix/4x1/double/random muxTest 4 1 "${TEST_DATA_DIR}/control/mux/mux-matrix-4-double-random.csv")
add_test(control/mux/matrix/2x2/double/random muxTest 2 2 "${TEST_DATA_DIR}/control/mux/mux-matrix-4-double-random.csv")
add_test(control/mux/matrix/5x5/double/random muxTest 5 5 "${TEST_DATA_DIR}/control/mux/mux-matrix-25-double-random.csv")
