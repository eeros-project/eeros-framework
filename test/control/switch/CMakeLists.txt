##### UNIT TESTS FOR SWITCH #####

# Compile and link test application
add_executable(switchTest SwitchTest.cpp)
target_link_libraries(switchTest eeros ${EEROS_LIBS})

# Add tests
add_test(control/switch/2input/integer/random switchTest 2)
add_test(control/switch/3input/integer/random switchTest 3)
add_test(control/switch/10input/integer/random switchTest 10)
