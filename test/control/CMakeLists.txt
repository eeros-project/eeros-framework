
##### UNIT TESTS FOR CONTROL SYSTEM #####

add_eeros_test_sources(Constant.cpp)
add_eeros_test_sources(DeMux.cpp)
add_eeros_test_sources(I.cpp)
add_eeros_test_sources(Mul.cpp)
add_eeros_test_sources(PathPlannerCubic.cpp)
add_eeros_test_sources(PathPlannerTrapezoid.cpp)
add_eeros_test_sources(SignalChecker.cpp)
add_eeros_test_sources(SocketData.cpp)
add_eeros_test_sources(Sum.cpp)
add_eeros_test_sources(Transition.cpp)
add_eeros_test_sources(MAFilter.cpp)
add_eeros_test_sources(MedianFilter.cpp)


# Blocks, old, delete when replaced by new tests
add_subdirectory(d)
add_subdirectory(mux)
add_subdirectory(switch)
add_subdirectory(saturation)
add_subdirectory(gain)
add_subdirectory(constant)
add_subdirectory(trajectoryGenerator)
