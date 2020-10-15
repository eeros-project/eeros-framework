add_executable(periodicExample1 PeriodicExample1.cpp)
target_link_libraries(periodicExample1 eeros ${EEROS_LIBS})

add_executable(periodicExample2 PeriodicExample2.cpp)
target_link_libraries(periodicExample2 eeros ${EEROS_LIBS})

add_executable(periodicExample3 PeriodicExample3.cpp)
target_link_libraries(periodicExample3 eeros ${EEROS_LIBS})
