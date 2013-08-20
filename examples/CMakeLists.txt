project(eeros-examples)

include_directories(${EEROS_SOURCE_DIR}/includes ${EEROS_BINARY_DIR})

add_subdirectory(sequencer)
add_subdirectory(simpleMotorController)