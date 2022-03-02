include(CMakeFindDependencyMacro)
include(${CMAKE_CURRENT_LIST_DIR}/EEROS.cmake)

get_target_property(eeros_interface_comp_defs eeros "INTERFACE_COMPILE_DEFINITIONS")

if("USE_ETHERCAT" IN_LIST eeros_interface_comp_defs)
  find_dependency(ecmasterlib)
endif()