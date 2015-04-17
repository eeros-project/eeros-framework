##### UNIT TESTS FOR TRANSITIONBLOCK #####

# Compile and link test application
add_executable(transitionTest TransitionTest.cpp)
target_link_libraries(transitionTest eeros)

# Add tests
add_test(control/transition transitionTest)
