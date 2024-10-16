include(cmake/package_management.cmake)

function(add_driver_option)
    set(noValues)
    set(singleValues FLAG_NAME HELP_TEXT)
    set(mutliValues PACKAGE_DEPENDS LINK_LIBRARIES LINK_LIBRARIES_PUBLIC PKG_CONFIG COMPILE_DEFINITIONS SOURCES CMAKE_FILES)
    cmake_parse_arguments(ARG "${noValues}" "${singleValues}" "${mutliValues}" ${ARGN})

    option(${ARG_FLAG_NAME} ${ARG_HELP_TEXT} OFF)

    if(${ARG_FLAG_NAME})
        target_compile_definitions(${PROJECT_NAME}_eeros PUBLIC EEROS_${ARG_FLAG_NAME})

        foreach(package IN LISTS ARG_PACKAGE_DEPENDS)
            message("looking for ${package}")
            eeros_find_package(${PROJECT_NAME}_eeros ${package})
        endforeach()

        if(DEFINED ARG_LINK_LIBRARIES)
            message("linking ${ARG_LINK_LIBRARIES}")
            target_link_libraries(${PROJECT_NAME}_eeros PRIVATE ${ARG_LINK_LIBRARIES})
        endif()

        if(DEFINED ARG_LINK_LIBRARIES_PUBLIC)
            message("linking public ${ARG_LINK_LIBRARIES_PUBLIC}")
            target_link_libraries(${PROJECT_NAME}_eeros PUBLIC "${ARG_LINK_LIBRARIES_PUBLIC}")
        endif()

        if(DEFINED ARG_PKG_CONFIG)
            include(FindPkgConfig)

            foreach(pkg IN LISTS ARG_PKG_CONFIG)
                eeros_find_pkgconfig(${PROJECT_NAME}_eeros "${pkg}")
                target_link_libraries(${PROJECT_NAME}_eeros PUBLIC "PkgConfig::${pkg}")
            endforeach()
        endif()

        if(DEFINED ARG_COMPILE_DEFINITIONS)
            message("adding compile definitions: ${ARG_COMPILE_DEFINITIONS}")
            target_compile_definitions(${PROJECT_NAME}_eeros PUBLIC ${ARG_COMPILE_DEFINITIONS})
        endif()

        if(DEFINED ARG_SOURCES)
            target_sources(${PROJECT_NAME}_eeros PRIVATE ${ARG_SOURCES})
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