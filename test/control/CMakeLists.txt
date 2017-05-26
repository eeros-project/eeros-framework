
##### UNIT TESTS FOR CONTROL SYSTEM #####

add_eeros_test_sources(DeMux.cpp)

# Blocks
add_subdirectory(sum)
add_subdirectory(d)
add_subdirectory(mux)
# add_subdirectory(demux)
add_subdirectory(switch)
add_subdirectory(saturation)
add_subdirectory(gain)
add_subdirectory(constant)
add_subdirectory(trajectoryGenerator)

