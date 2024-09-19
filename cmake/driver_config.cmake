
function(add_driver_option)
    set(noValues)
    set(singleValues FLAG_NAME HELP_TEXT)
    set(mutliValues PACKAGE_DEPENDS LINK_LIBRARIES COMPILE_DEFINITIONS CMAKE_FILES)
    cmake_parse_arguments(ARG "${noValues}" "${singleValues}" "${mutliValues}" ${ARGN})

    option(${ARG_FLAG_NAME} ${ARG_HELP_TEXT} OFF)

    if(${ARG_FLAG_NAME})
        foreach(package IN LISTS ARG_PACKAGE_DEPENDS)
            message("looking for ${package}")
            find_package(${package} REQUIRED)
        endforeach()

        if(DEFINED ARG_LINK_LIBRARIES)
            message("linking ${ARG_LINK_LIBRARIES}")
            target_link_libraries(eeros PUBLIC ${ARG_LINK_LIBRARIES})
        elseif(DEFINED ARG_PACKAGE_DEPENDS)
            # fallback for convenience if cmake package name == library name
            target_link_libraries(eeros PUBLIC ${ARG_PACKAGE_DEPENDS})
        endif()

        if(DEFINED ARG_COMPILE_DEFINITIONS)
            message("adding compile definitions: ${ARG_COMPILE_DEFINITIONS}")
            target_compile_definitions(eeros PUBLIC ${ARG_COMPILE_DEFINITIONS})
        endif()

        foreach(file IN LISTS ARG_CMAKE_FILES)
            message(DEBUG "including ${file}")
            include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/driver_config/${file})
        endforeach()

        message("option \"${ARG_HELP_TEXT}\" enabled")
    else()
        message("option \"${ARG_HELP_TEXT}\" not enabled")
    endif()
endfunction()

option(name "help" default)