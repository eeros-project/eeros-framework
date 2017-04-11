
##### UNIT TESTS FOR SUM #####

# Compile and link test application
add_executable(sum Sum.cpp)
target_link_libraries(sum eeros ${EEROS_LIBS})

# Add tests
add_test(control/sum/double/2inputs/random sum 2 "${TEST_DATA_DIR}/control/sum/sum-double-2inputs-random.csv")
add_test(control/sum/double/3inputs/random sum 3 "${TEST_DATA_DIR}/control/sum/sum-double-3inputs-random.csv")
add_test(control/sum/double/10inputs/random sum 10 "${TEST_DATA_DIR}/control/sum/sum-double-10inputs-random.csv")
add_test(control/sum/matrix/4x1/2inputs/random sum 2 4 1 "${TEST_DATA_DIR}/control/sum/sum-matrix4-2inputs-random.csv")
add_test(control/sum/matrix/2x2/2inputs/random sum 2 2 2 "${TEST_DATA_DIR}/control/sum/sum-matrix4-2inputs-random.csv")
