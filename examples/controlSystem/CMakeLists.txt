add_custom_command(OUTPUT copyControlSystemConfig POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/controlSystem/BlockTest2Config.json" "${EEROS_BINARY_DIR}/examples/controlSystem")

add_executable(periodicExample1 PeriodicExample1.cpp)
target_link_libraries(periodicExample1 eeros ${EEROS_LIBS})

add_executable(periodicExample2 PeriodicExample2.cpp)
target_link_libraries(periodicExample2 eeros ${EEROS_LIBS})

add_executable(blockTest1 BlockTest1.cpp)
target_link_libraries(blockTest1 eeros ${EEROS_LIBS})

add_executable(blockTest2 BlockTest2.cpp copyControlSystemConfig)
target_link_libraries(blockTest2 eeros ${EEROS_LIBS})

add_executable(blockTest3 BlockTest3.cpp)
target_link_libraries(blockTest3 eeros ${EEROS_LIBS})

add_executable(mouseTest MouseTest.cpp)
target_link_libraries(mouseTest eeros ${EEROS_LIBS})

add_executable(xBoxTest XBoxTest.cpp)
target_link_libraries(xBoxTest eeros ${EEROS_LIBS})
