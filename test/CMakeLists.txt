
##### UNIT TESTS #####

include_directories(${EEROS_SOURCE_DIR}/includes ${EEROS_BINARY_DIR})

set(TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/test/data)

# Framework components

add_subdirectory(core)
add_subdirectory(math)
add_subdirectory(control)
add_subdirectory(safety)
add_subdirectory(hal)