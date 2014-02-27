
##### UNIT TESTS FOR SUM #####

# Compile and link test application
add_executable(sum Sum.cpp)
target_link_libraries(sum eeros)

# Add tests
add_test(control/sum/double/2inputs/random sum 2 "${TEST_DATA_DIR}/control/sum/sum-double-2inputs-random.csv")
add_test(control/sum/double/3inputs/random sum 3 "${TEST_DATA_DIR}/control/sum/sum-double-3inputs-random.csv")
add_test(control/sum/double/10inputs/random sum 10 "${TEST_DATA_DIR}/control/sum/sum-double-10inputs-random.csv")
