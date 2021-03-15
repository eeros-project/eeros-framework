add_custom_command(OUTPUT copyPathFile POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/block/path.txt" "${EEROS_BINARY_DIR}/examples/block")

set(targets "")

add_executable(blockTest BlockTest.cpp)
target_link_libraries(blockTest eeros ${EEROS_LIBS})
list(APPEND targets blockTest)

add_executable(mouseTest MouseTest.cpp)
target_link_libraries(mouseTest eeros ${EEROS_LIBS})
list(APPEND targets mouseTest)

add_executable(xBoxTest XBoxTest.cpp)
target_link_libraries(xBoxTest eeros ${EEROS_LIBS})
list(APPEND targets xBoxTest)

add_executable(keyboardTest KeyboardTest.cpp)
target_link_libraries(keyboardTest eeros ${EEROS_LIBS})
list(APPEND targets keyboardTest)

add_executable(traceTest TraceTest.cpp)
target_link_libraries(traceTest eeros ${EEROS_LIBS})
list(APPEND targets traceTest)

add_executable(spaceNavigatorTest SpaceNavigatorTest.cpp)
target_link_libraries(spaceNavigatorTest eeros ${EEROS_LIBS})
list(APPEND targets spaceNavigatorTest)

add_executable(transitionBlockTest1 TransitionBlockTest.cpp)
target_link_libraries(transitionBlockTest1 eeros ${EEROS_LIBS})
list(APPEND targets transitionBlockTest1)

add_executable(pathPlannerTest1 PathPlannerTest1.cpp)
target_link_libraries(pathPlannerTest1 eeros ${EEROS_LIBS})
list(APPEND targets pathPlannerTest1)

add_executable(pathPlannerTest2 PathPlannerTest2.cpp)
target_link_libraries(pathPlannerTest2 eeros ${EEROS_LIBS})
list(APPEND targets pathPlannerTest2)

add_executable(pathPlannerTest3 PathPlannerTest3.cpp copyPathFile)
target_link_libraries(pathPlannerTest3 eeros ${EEROS_LIBS})
list(APPEND targets pathPlannerTest3)


if(INSTALL_EXAMPLES)
  install(TARGETS ${targets} RUNTIME DESTINATION examples/block)
  install(FILES ${EEROS_BINARY_DIR}/examples/block/path.txt DESTINATION examples/block)
endif()

