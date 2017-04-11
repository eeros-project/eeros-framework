##### UNIT TESTS FOR CONSTANT BLOCK #####

# Compile and link test application
add_executable(constantTestArithmetic ConstantTestArithmetic.cpp)
target_link_libraries(constantTestArithmetic eeros ${EEROS_LIBS})

add_executable(constantTestContainer ConstantTestContainer.cpp)
target_link_libraries(constantTestContainer eeros ${EEROS_LIBS})

# Add tests
add_test(control/constant/int/random               constantTestArithmetic i)
add_test(control/constant/long/random              constantTestArithmetic l)
add_test(control/constant/float/random             constantTestArithmetic f)
add_test(control/constant/double/random            constantTestArithmetic d)

add_test(control/constant/matrix/3x1/int/random    constantTestContainer m 3 1 i)
add_test(control/constant/matrix/1x2/long/random   constantTestContainer m 1 2 l)
add_test(control/constant/matrix/2x2/float/random  constantTestContainer m 2 2 f)
add_test(control/constant/matrix/3x2/double/random constantTestContainer m 3 2 d)

add_test(control/constant/stdarray/7/int/random    constantTestContainer a 7 i)
add_test(control/constant/stdarray/3/long/random   constantTestContainer a 3 l)
add_test(control/constant/stdarray/5/float/random  constantTestContainer a 5 f)
add_test(control/constant/stdarray/4/double/random constantTestContainer a 4 d)
