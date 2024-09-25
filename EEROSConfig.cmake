include(CMakeFindDependencyMacro)
include(FindPkgConfig)

include(${CMAKE_CURRENT_LIST_DIR}/EEROS.cmake)

get_target_property(eers_cmake_package_depends EEROS::eeros "EEROS_CMAKE_PACKAGE_DEPENDS")

if(eers_cmake_package_depends)
  foreach(dependency IN LISTS eers_cmake_package_depends)
    message("found EEROS dep: ${dependency}")
    find_dependency(${dependency})
  endforeach()
endif()

get_target_property(eeros_pkgconfig_depends EEROS::eeros "EEROS_PKGCONFIG_DEPENDS")

if(eeros_pkgconfig_depends)
  foreach(dependency IN LISTS eeros_pkgconfig_depends)
    message("found EEROS pkgconfig dep: ${dependency}")
    pkg_search_module("${dependency}" IMPORTED_TARGET REQUIRED ${dependency})
  endforeach()
endif()