##### UNIT TESTS FOR COSINUS TRAJECTORY GENERATOR #####

# Compile and link test application
add_executable(constantAccTrajectoryGeneratorTestVector1 ConstantAccTrajectoryGeneratorTestVector1.cpp)
target_link_libraries(constantAccTrajectoryGeneratorTestVector1 eeros pthread)

add_executable(constantAccTrajectoryGeneratorTestVector4 ConstantAccTrajectoryGeneratorTestVector4.cpp)
target_link_libraries(constantAccTrajectoryGeneratorTestVector4 eeros pthread)

add_test(control/trajectoryGenerator/constantAcc/vector1 constantAccTrajectoryGeneratorTestVector1 "${TEST_DATA_DIR}/control/trajectoryGenerator/constantAccTrajectoryGenerator1.txt")
add_test(control/trajectoryGenerator/constantAcc/vector4 constantAccTrajectoryGeneratorTestVector4 "${TEST_DATA_DIR}/control/trajectoryGenerator/constantAccTrajectoryGenerator4.txt")

