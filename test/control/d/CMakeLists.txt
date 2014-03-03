##### UNIT TESTS FOR D #####

# Compile and link test application
add_executable(d D.cpp)
target_link_libraries(d eeros)

# Add tests
add_test(control/d/double/random d "${TEST_DATA_DIR}/control/d/d-double-random.csv")
