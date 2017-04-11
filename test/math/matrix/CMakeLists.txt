##### UNIT TESTS FOR MATRIX CLASS #####

# Compile and link test applications
add_executable(elementAccess ElementAccess.cpp)
target_link_libraries(elementAccess eeros pthread ucl)

add_executable(initialization Initialization.cpp)
target_link_libraries(initialization eeros pthread ucl)

add_executable(relationalOperators RelationalOperators.cpp)
target_link_libraries(relationalOperators eeros pthread ucl)

add_executable(characteristics Characteristics.cpp)
target_link_libraries(characteristics eeros pthread ucl)

add_executable(scalarOperations ScalarOperations.cpp)
target_link_libraries(scalarOperations eeros pthread ucl)

add_executable(matrixOperations MatrixOperations.cpp)
target_link_libraries(matrixOperations eeros pthread ucl)

add_executable(matrixOperations2 MatrixOperations2.cpp)
target_link_libraries(matrixOperations2 eeros pthread ucl)

# Add tests
add_test(math/Matrix/elementAccess elementAccess)
add_test(math/Matrix/initialization initialization "${TEST_DATA_DIR}/math/matrix/matrix-rotx.csv" "${TEST_DATA_DIR}/math/matrix/matrix-roty.csv" "${TEST_DATA_DIR}/math/matrix/matrix-rotz.csv")
add_test(math/Matrix/relational-operators relationalOperators)
add_test(math/Matrix/characteristics characteristics)
add_test(math/Matrix/scalar-operations/4x1/double/random scalarOperations 4 1 "${TEST_DATA_DIR}/math/matrix/matrix-scalar-ops-4-double-random.csv")
add_test(math/Matrix/scalar-operations/1x4/double/random scalarOperations 1 4 "${TEST_DATA_DIR}/math/matrix/matrix-scalar-ops-4-double-random.csv")
add_test(math/Matrix/scalar-operations/2x2/double/random scalarOperations 2 2 "${TEST_DATA_DIR}/math/matrix/matrix-scalar-ops-4-double-random.csv")
add_test(math/Matrix/scalar-operations/3x2/double/random scalarOperations 3 2 "${TEST_DATA_DIR}/math/matrix/matrix-scalar-ops-6-double-random.csv")
add_test(math/Matrix/matrix-operations/1x3/double/random matrixOperations 2 2 2 2 "${TEST_DATA_DIR}/math/matrix/test22.dat")
add_test(math/Matrix/matrix-operations/3x3/double/random matrixOperations 3 3 3 3 "${TEST_DATA_DIR}/math/matrix/test33.dat")
add_test(math/Matrix/matrix-operations/4x4/double/random matrixOperations 4 4 4 4 "${TEST_DATA_DIR}/math/matrix/test44.dat")
add_test(math/Matrix/matrix-operations/misc matrixOperations2)
