# eeros_add_dependent_target(odriveUSBTest DEPENDS_ON USE_ODRIVE SOURCES)

function(eeros_add_target target)
    add_executable("${PROJECT_NAME}_${target}" ${ARGN})
    # preserve old naming for examples, since they are installed under ${PROJECT_NAME} anyway
    set_target_properties("${PROJECT_NAME}_${target}" PROPERTIES OUTPUT_NAME "${target}")
    target_link_libraries("${PROJECT_NAME}_${target}" PRIVATE ${PROJECT_NAME}::eeros)
    message("current list dir: ${CMAKE_CURRENT_LIST_DIR}, project dir: ${PROJECT_SOURCE_DIR}")
    cmake_path(RELATIVE_PATH CMAKE_CURRENT_LIST_DIR BASE_DIRECTORY "${PROJECT_SOURCE_DIR}" OUTPUT_VARIABLE "${target}_relpath")
    message("path for ${target}: ${${target}_relpath}")
    install(TARGETS "${PROJECT_NAME}_${target}" RUNTIME DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/${${target}_relpath})
endfunction()

function(eeros_add_dependent_target target)
    cmake_parse_arguments(ARG "" "DEPENDS_ON" "SOURCES" ${ARGN})

    if(${ARG_DEPENDS_ON})
        eeros_add_target("${target}" ${ARG_SOURCES})
    else()
        message("${target} not enabled")
    endif()
endfunction()

function(eeros_copy_file_post_build commandName filename)
    cmake_path(RELATIVE_PATH CMAKE_CURRENT_LIST_DIR BASE_DIRECTORY "${PROJECT_SOURCE_DIR}" OUTPUT_VARIABLE copy_relpath)
    set("${commandName}_buildpath" "${PROJECT_BINARY_DIR}/${copy_relpath}/${filename}")
    message("${commandName}_buildpath: ${${commandName}_buildpath}")
    add_custom_command(OUTPUT "${commandName}" POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_LIST_DIR}/${filename}" "${${commandName}_buildpath}")

    if(EXISTS "${${commandName}_buildpath}")
        install(FILES "${${commandName}_buildpath}" DESTINATION "${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/${copy_relpath}")
    endif()
endfunction()