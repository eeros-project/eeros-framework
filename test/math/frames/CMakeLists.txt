##### UNIT TESTS FOR FRAMES CLASS #####

# Compile and link test applications
add_executable(coordinateSystemTest CoordinateSysTest.cpp)
target_link_libraries(coordinateSystemTest eeros ${EEROS_LIBS})

add_executable(frameTest FrameTest.cpp)
target_link_libraries(frameTest eeros ${EEROS_LIBS})


# Add tests
add_test(math/Frames/coordinateSystem coordinateSystemTest)
add_test(math/Frames/frame frameTest)
