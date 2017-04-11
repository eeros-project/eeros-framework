##### UNIT TESTS FOR DEMUX #####

# Compile and link test application
add_executable(demux DeMux.cpp)
target_link_libraries(demux eeros ucl)

# Add tests
add_test(control/demux/matrix/1x1/double/random demux 1 1 "${TEST_DATA_DIR}/control/demux/demux-matrix-1-double-random.csv")
add_test(control/demux/matrix/3x1/double/random demux 3 1 "${TEST_DATA_DIR}/control/demux/demux-matrix-3-double-random.csv")
add_test(control/demux/matrix/4x1/double/random demux 4 1 "${TEST_DATA_DIR}/control/demux/demux-matrix-4-double-random.csv")
add_test(control/demux/matrix/2x2/double/random demux 2 2 "${TEST_DATA_DIR}/control/demux/demux-matrix-4-double-random.csv")
add_test(control/demux/matrix/5x5/double/random demux 5 5 "${TEST_DATA_DIR}/control/demux/demux-matrix-25-double-random.csv")
