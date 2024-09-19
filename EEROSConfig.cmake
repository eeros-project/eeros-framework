include(CMakeFindDependencyMacro)
include(${CMAKE_CURRENT_LIST_DIR}/EEROS.cmake)

get_target_property(eeros_interface_comp_defs EEROS::eeros "INTERFACE_COMPILE_DEFINITIONS")

if("USE_ETHERCAT" IN_LIST eeros_interface_comp_defs)
  find_dependency(ecmasterlib)
endif()

get_target_property(eeros_interface_libs EEROS::eeros "INTERFACE_LINK_LIBRARIES")

foreach(dependency IN LISTS eeros_interface_libs)
  message("found EEROS dep: ${dependency}")
  string(FIND ${dependency} :: package_name_end)
  string(SUBSTRING ${dependency} 0 ${package_name_end} dep_package)
  message("package name: ${dep_package}")
  find_dependency(${dep_package})
endforeach()