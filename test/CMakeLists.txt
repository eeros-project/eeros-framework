
##### UNIT TESTS #####

include_directories(${EEROS_SOURCE_DIR}/includes ${EEROS_BINARY_DIR} ${EEROS_SOURCE_DIR}/test)

set(TEST_DATA_DIR ${CMAKE_SOURCE_DIR}/test/data)

##########################
# load and add googletest
##########################
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

#######################
# EEROS test sources
#######################

macro(add_eeros_test_sources)
    file(RELATIVE_PATH _relPath "${PROJECT_SOURCE_DIR}/test" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach(_test ${ARGN})
        if(_relPath)
            list(APPEND EEROS_TEST_SRCS "${_relPath}/${_test}")
        else()
            list(APPEND EEROS_TEST_SRCS "${_test}")
        endif()
    endforeach()
    if(_relPath)
        # propagate EEROS_TEST_SRCS to parent directory
        set(EEROS_TEST_SRCS ${EEROS_TEST_SRCS} PARENT_SCOPE)
    endif()
endmacro()

#add_subdirectory(core)
#add_subdirectory(math)
#add_subdirectory(control)
#add_subdirectory(safety)
add_subdirectory(hal)

add_eeros_test_sources(RunAllTests.cpp)
add_eeros_test_sources(EerosEnvironment.cpp)
add_eeros_test_sources(EerosEnvironmentInvalidConfig.cpp)

add_executable(unitTests ${EEROS_TEST_SRCS})
target_link_libraries(unitTests ${EXTERNAL_LIBS} eeros dl gtest_main)
add_test(NAME eeros_unit_tests COMMAND unitTests)

set( HAL_CONFIG_FILES
      ${EEROS_SOURCE_DIR}/test/hal/loadConfigComedi.json
      ${EEROS_SOURCE_DIR}/test/hal/loadConfigFlink.json
      ${EEROS_SOURCE_DIR}/test/hal/loadConfigSim.json
      ${EEROS_SOURCE_DIR}/test/hal/invalidScaleComedi.json
      ${EEROS_SOURCE_DIR}/test/hal/invalidScaleFlink.json
      ${EEROS_SOURCE_DIR}/test/hal/invalidScaleSim.json
      ${EEROS_SOURCE_DIR}/test/hal/invalidChannelComedi.json
      ${EEROS_SOURCE_DIR}/test/hal/invalidChannelFlink.json
      ${EEROS_SOURCE_DIR}/test/hal/invalidChannelSim.json
      ${EEROS_SOURCE_DIR}/test/hal/invalidTypeComedi.json
      ${EEROS_SOURCE_DIR}/test/hal/invalidTypeFlink.json
      ${EEROS_SOURCE_DIR}/test/hal/invalidTypeSim.json )
      
foreach( halConfig_i ${HAL_CONFIG_FILES})
          add_custom_command(TARGET unitTests POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${halConfig_i} "${EEROS_BINARY_DIR}/test/")
endforeach( halConfig_i )
