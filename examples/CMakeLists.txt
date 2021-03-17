include_directories(${EEROS_SOURCE_DIR}/includes ${EEROS_BINARY_DIR})

set(INSTALL_EXAMPLES TRUE CACHE BOOL "Install EEROS examples if TRUE") # default if not set in CACHE.

add_subdirectory(config)
add_subdirectory(block)
add_subdirectory(task)
add_subdirectory(hal)
add_subdirectory(logger)
add_subdirectory(ros)
add_subdirectory(can)
add_subdirectory(rtTest)
add_subdirectory(sequencer)
add_subdirectory(socket)
add_subdirectory(devel)
add_subdirectory(system)

