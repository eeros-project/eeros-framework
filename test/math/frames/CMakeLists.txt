##### UNIT TESTS FOR FRAMES CLASS #####

# Compile and link test applications
add_executable(coordinateSystemTest CoordinateSysTest.cpp)
target_link_libraries(coordinateSystemTest eeros)

# Add tests
add_test(math/Frames/coordinateSystem coordinateSystemTest)
