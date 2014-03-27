##### UNIT TESTS FOR MATRIX CLASS #####

# Compile and link test applications
add_executable(elementAccess ElementAccess.cpp)
target_link_libraries(elementAccess eeros)

add_executable(initialization Initialization.cpp)
target_link_libraries(initialization eeros)

add_executable(characteristics Characteristics.cpp)
target_link_libraries(characteristics eeros)

add_executable(operations Operations.cpp)
target_link_libraries(operations eeros)

# Add tests
add_test(math/Matrix/elementAccess elementAccess)
add_test(math/Matrix/initialization initialization "${TEST_DATA_DIR}/math/matrix/matrix-rotx.csv" "${TEST_DATA_DIR}/math/matrix/matrix-roty.csv" "${TEST_DATA_DIR}/math/matrix/matrix-rotz.csv")
add_test(math/Matrix/characteristics characteristics)
add_test(math/Matrix/operations/2x2/random operations 2 2 2 2 "${TEST_DATA_DIR}/math/matrix/test22.dat")
add_test(math/Matrix/operations/3x3/random operations 3 3 3 3 "${TEST_DATA_DIR}/math/matrix/test33.dat")
add_test(math/Matrix/operations/4x4/random operations 4 4 4 4 "${TEST_DATA_DIR}/math/matrix/test44.dat")
