##### UNIT TESTS FOR SATURATION #####

# Compile and link test application
add_executable(saturationTest SaturationTest.cpp)
target_link_libraries(saturationTest eeros)

# Add tests
add_test(control/saturation/int/random saturationTest i)
add_test(control/saturation/double/random saturationTest d)
# add_test(control/saturation/matrix/3x1/double/random saturationTest v)
# add_test(control/saturation/matrix/4x4/double/random saturationTest m)
