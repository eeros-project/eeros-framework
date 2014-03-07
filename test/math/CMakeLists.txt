# Compile and link test application
add_executable(matrixTest MatrixTest.cpp)
target_link_libraries(matrixTest eeros)

add_executable(matrixOperationTest MatrixOperationTest.cpp)
target_link_libraries(matrixOperationTest eeros)


# Add tests
add_test(math/Matrix/accessAndCharacteristics/random matrixTest )
add_test(math/Matrix/operations/2x2/random matrixOperationTest 2 2 2 2 "${TEST_DATA_DIR}/math/matrixOperations/test22.dat")
add_test(math/Matrix/operations/3x3/random matrixOperationTest 3 3 3 3 "${TEST_DATA_DIR}/math/matrixOperations/test33.dat")
add_test(math/Matrix/operations/4x4/random matrixOperationTest 4 4 4 4 "${TEST_DATA_DIR}/math/matrixOperations/test44.dat")
