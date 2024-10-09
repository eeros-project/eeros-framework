# # Load CMake modules library at specified version
set(LOAD_CMAKE_MODULES_LIB TRUE CACHE BOOL "TRUE if the cmake-modules library should be loaded.")

if(LOAD_CMAKE_MODULES_LIB)
    include(cmake/CMakeModulesLibLoader.cmake)
    load_cmake_modules_lib(https://github.com/eeros-project/cmake-modules.git)
endif()

include(CMakeModulesLib)

if(LOAD_CMAKE_MODULES_LIB)
    checkout_cmake_modules_lib_version(a50add2)
endif()