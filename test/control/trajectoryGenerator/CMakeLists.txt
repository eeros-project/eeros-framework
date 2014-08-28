##### UNIT TESTS FOR COSINUS TRAJECTORY GENERATOR #####

# Compile and link test application
add_executable(constantAccTrajectoryGeneratorTest ConstantAccTrajectoryGeneratorTest.cpp)
target_link_libraries(constantAccTrajectoryGeneratorTest eeros)

add_test(control/trajectoryGenerator/constantAcc constantAccTrajectoryGeneratorTest "${TEST_DATA_DIR}/control/trajectoryGenerator/constantAccTrajectoryGenerator.txt")

