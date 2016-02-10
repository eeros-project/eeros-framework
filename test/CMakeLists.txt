
##### UNIT TESTS #####

include_directories(${EEROS_SOURCE_DIR}/includes ${EEROS_BINARY_DIR})

set(TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/test/data)

# Framework components

add_subdirectory(core)
add_subdirectory(math)
add_subdirectory(control)
add_subdirectory(safety)
#add_subdirectory(sequencer)
#add_subdirectory(logger)


# Misc tests of the EEROS developers

add_subdirectory(martin)
#add_subdirectory(vinci)
