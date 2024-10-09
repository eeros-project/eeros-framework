include(FindPkgConfig)

function(eeros_find_package target package)
    find_package("${package}" REQUIRED)
    eeros_add_package(${target} ${package})
endfunction()

function(eeros_add_package target package)
    set_property(TARGET "${target}" APPEND PROPERTY EXPORT_PROPERTIES EEROS_CMAKE_PACKAGE_DEPENDS)
    set_property(TARGET "${target}" APPEND PROPERTY EEROS_CMAKE_PACKAGE_DEPENDS "${package}")
endfunction()

function(eeros_find_pkgconfig target package)
    message("looking for ${package}")
    set_property(TARGET "${target}" APPEND PROPERTY EXPORT_PROPERTIES EEROS_PKGCONFIG_DEPENDS)
    pkg_search_module("${package}" IMPORTED_TARGET REQUIRED ${package})
    set_property(TARGET "${target}" APPEND PROPERTY EEROS_PKGCONFIG_DEPENDS "${package}")
endfunction()
