
macro (add_eeros_sources)
    file (RELATIVE_PATH _relPath "${CMAKE_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach (_src ${ARGN})
        if (_relPath)
            list (APPEND EEROS_SRCS "${_relPath}/${_src}")
        else()
            list (APPEND EEROS_SRCS "${_src}")
        endif()
    endforeach()
    if (_relPath)
        # propagate EEROS_SRCS to parent directory
        set (EEROS_SRCS ${EEROS_SRCS} PARENT_SCOPE)
    endif()
endmacro()

include_directories(${EEROS_SOURCE_DIR}/includes)

add_subdirectory(core)
add_subdirectory(control)
add_subdirectory(hal)

link_directories (${EEROS_BINARY_DIR}/src/core)
link_directories (${EEROS_BINARY_DIR}/src/control)
link_directories (${EEROS_BINARY_DIR}/src/hal)

add_library(eeros SHARED ${EEROS_SRCS})