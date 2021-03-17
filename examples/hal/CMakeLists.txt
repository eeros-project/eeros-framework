set(files "")
add_custom_command(OUTPUT copyHalTest1Config POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/hal/HalTest1Config.json" "${EEROS_BINARY_DIR}/examples/hal")
list(APPEND files ${EEROS_BINARY_DIR}/examples/hal/HalTest1Config.json)

add_custom_command(OUTPUT copyHalTest2CConfig POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/hal/HalTest2ConfigComedi.json" "${EEROS_BINARY_DIR}/examples/hal")
list(APPEND files ${EEROS_BINARY_DIR}/examples/hal/HalTest2ConfigComedi.json)

add_custom_command(OUTPUT copyHalTest2FConfig POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${EEROS_SOURCE_DIR}/examples/hal/HalTest2ConfigFlink.json" "${EEROS_BINARY_DIR}/examples/hal")
list(APPEND files ${EEROS_BINARY_DIR}/examples/hal/HalTest2ConfigFlink.json)


set(targets "")

add_executable(halTest1 HalTest1.cpp copyHalTest1Config)
target_link_libraries(halTest1 eeros ${EEROS_LIBS})
list(APPEND targets halTest1)

add_executable(halTest2 HalTest2.cpp copyHalTest2CConfig copyHalTest2FConfig)
target_link_libraries(halTest2 eeros ${EEROS_LIBS})
list(APPEND targets halTest2)


if(INSTALL_EXAMPLES)
  install(TARGETS ${targets} RUNTIME DESTINATION examples/hal)
  install(FILES ${files} DESTINATION examples/hal)
endif()

