set(targets "")

add_executable(lowPassFilterTest LowPassFilterTest.cpp)
target_link_libraries(lowPassFilterTest eeros ${EEROS_LIBS})
list(APPEND targets lowPassFilterTest)

add_executable(movingAverageFilterTest MovingAverageFilterTest.cpp)
target_link_libraries(movingAverageFilterTest eeros ${EEROS_LIBS})
list(APPEND targets movingAverageFilterTest)

add_executable(medianFilterTest MedianFilterTest.cpp)
target_link_libraries(medianFilterTest eeros ${EEROS_LIBS})
list(APPEND targets medianFilterTest)

if(INSTALL_EXAMPLES)
  install(TARGETS ${targets} RUNTIME DESTINATION examples/block/filter)
endif()
