
add_executable(sum Sum.cpp)
target_link_libraries(sum eeros)
add_test(control/sum/random sum "${CMAKE_CURRENT_SOURCE_DIR}/sum-random.csv")
