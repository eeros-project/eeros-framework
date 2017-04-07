
macro(add_eeros_sources)
    file(RELATIVE_PATH _relPath "${PROJECT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach(_src ${ARGN})
        if(_relPath)
            list(APPEND EEROS_SRCS "${_relPath}/${_src}")
        else()
            list(APPEND EEROS_SRCS "${_src}")
        endif()
    endforeach()
    if(_relPath)
        # propagate EEROS_SRCS to parent directory
        set(EEROS_SRCS ${EEROS_SRCS} PARENT_SCOPE)
    endif()
endmacro()

include_directories(${EEROS_SOURCE_DIR}/includes ${EEROS_BINARY_DIR})

add_subdirectory(core)
add_subdirectory(task)
add_subdirectory(math)
add_subdirectory(control)
add_subdirectory(hal)
add_subdirectory(sequencer)
add_subdirectory(sockets)
add_subdirectory(safety)
add_subdirectory(logger)
add_subdirectory(ui)

link_directories(${EEROS_BINARY_DIR}/src/core)
link_directories(${EEROS_BINARY_DIR}/src/math)
link_directories(${EEROS_BINARY_DIR}/src/control)
link_directories(${EEROS_BINARY_DIR}/src/hal)
link_directories(${EEROS_BINARY_DIR}/src/sequencer)
link_directories(${EEROS_BINARY_DIR}/src/sockets)
link_directories(${EEROS_BINARY_DIR}/src/safety)
link_directories(${EEROS_BINARY_DIR}/src/logger)
link_directories(${EEROS_BINARY_DIR}/src/ui)

add_library(eeros SHARED ${EEROS_SRCS})

target_link_libraries(eeros ${EXTERNAL_LIBS})

set_target_properties(eeros PROPERTIES
			VERSION ${EEROS_VERSION}
			SOVERSION ${EEROS_VERSION_SO})


INSTALL(TARGETS eeros LIBRARY DESTINATION lib)
