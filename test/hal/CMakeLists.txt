
add_executable(parserTest parserTest.cpp)
target_link_libraries(parserTest eeros)
add_test(hal/parser parserTest)
