##### UNIT TESTS FOR SATURATION #####

# Compile and link test application
add_executable(saturationTestArithmetic SaturationTestArithmetic.cpp)
target_link_libraries(saturationTestArithmetic eeros ${EEROS_LIBS})

add_executable(saturationTestContainer SaturationTestContainer.cpp)
target_link_libraries(saturationTestContainer eeros ${EEROS_LIBS})

# Add tests
add_test(control/saturation/int/random    saturationTestArithmetic i)
add_test(control/saturation/long/random   saturationTestArithmetic l)
add_test(control/saturation/float/random  saturationTestArithmetic f)
add_test(control/saturation/double/random saturationTestArithmetic d)

add_test(control/saturation/matrix/3x1/int/random    saturationTestContainer m 3 1 i)
add_test(control/saturation/matrix/1x2/long/random   saturationTestContainer m 1 2 l)
add_test(control/saturation/matrix/2x2/float/random  saturationTestContainer m 2 2 f)
add_test(control/saturation/matrix/3x2/double/random saturationTestContainer m 3 2 d)

add_test(control/saturation/stdarray/7/int/random    saturationTestContainer a 7 i)
add_test(control/saturation/stdarray/3/long/random   saturationTestContainer a 3 l)
add_test(control/saturation/stdarray/5/float/random  saturationTestContainer a 5 f)
add_test(control/saturation/stdarray/4/double/random saturationTestContainer a 4 d)
