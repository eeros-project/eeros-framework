##### UNIT TESTS FOR GAIN #####

# Compile and link test applications
add_executable(gainTestScalar GainTestScalar.cpp)
target_link_libraries(gainTestScalar eeros)

add_executable(gainTestMatrix GainTestMatrix.cpp)
target_link_libraries(gainTestMatrix eeros)

# Add tests
add_test(control/gain/scalar-scalar/double/random gainTestScalar d "${TEST_DATA_DIR}/control/gain/gain-scalar-scalar-double-random.csv")
add_test(control/gain/scalar-scalar/int/random    gainTestScalar i "${TEST_DATA_DIR}/control/gain/gain-scalar-scalar-int-random.csv")
add_test(control/gain/3x1-3x1/double/random       gainTestMatrix 3 1 3 1 "${TEST_DATA_DIR}/control/gain/gain-matrix-3x1-3x1-double-random.csv")
add_test(control/gain/3x1-3x3/double/random       gainTestMatrix 3 1 3 3 "${TEST_DATA_DIR}/control/gain/gain-matrix-3x1-3x3-double-random.csv")
add_test(control/gain/2x2-2x2/double/random       gainTestMatrix 2 2 2 2 "${TEST_DATA_DIR}/control/gain/gain-matrix-2x2-2x2-double-random.csv")
#add_test(control/gain/3x2-2x4/double/random       gainTestMatrix 3 2 2 4 "${TEST_DATA_DIR}/control/gain/gain-matrix-3x2-2x4-random.csv")
