
##### UNIT TESTS #####

include_directories(${EEROS_SOURCE_DIR}/includes ${EEROS_BINARY_DIR})

set(TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/test/data)

# load googletest
# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in ${EEROS_BINARY_DIR}/test/googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${EEROS_BINARY_DIR}/test/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${EEROS_BINARY_DIR}/test/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${EEROS_BINARY_DIR}/test/googletest-src
                 ${EEROS_BINARY_DIR}/test/googletest-build)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES "googletest-download" "googletest-src" "googletest-build")

# Framework components

#add_subdirectory(core)
#add_subdirectory(math)
#add_subdirectory(control)
#add_subdirectory(safety)
add_subdirectory(hal)
