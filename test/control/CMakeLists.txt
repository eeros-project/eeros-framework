
##### UNIT TESTS FOR CONTROL SYSTEM #####

# Sum Block

add_executable(sum Sum.cpp)
target_link_libraries(sum eeros)

add_test(control/sum/2/random sum 2 "${CMAKE_CURRENT_SOURCE_DIR}/sum-random-2.csv")
add_test(control/sum/3/random sum 3 "${CMAKE_CURRENT_SOURCE_DIR}/sum-random-3.csv")
add_test(control/sum/10/random sum 10 "${CMAKE_CURRENT_SOURCE_DIR}/sum-random-10.csv")

# Gain Block