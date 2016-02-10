
add_executable(controlInputTest ControlInputTest.cpp)
target_link_libraries(controlInputTest eeros pthread)
add_test(core/controlInput controlInputTest)

