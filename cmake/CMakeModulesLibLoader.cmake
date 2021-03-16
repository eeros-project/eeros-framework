cmake_minimum_required(VERSION 3.10)

if(__cmake_modules_lib_loader)
  return()
endif()
set(__cmake_modules_lib_loader YES)


function(load_cmake_modules_lib)
  find_package(Git)
  if(NOT GIT_FOUND)
    message(FATAL_ERROR "git not found!")
  endif()


  if(NOT LIB_CMAKE_MODULES_EXISTS)
    set(LIB_CMAKE_MODULES_URL https://github.com/andreaskunz/cmake-modules.git CACHE STRING "The path to the origin of the cmake-modules library")

    execute_process(COMMAND ${GIT_EXECUTABLE} clone ${LIB_CMAKE_MODULES_URL}
                    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                    RESULT_VARIABLE LIB_CMAKE_MODULES_CLONE_RESULT
                    ERROR_VARIABLE LIB_CMAKE_MODULES_CLONE_OUTPUT)
    if(NOT LIB_CMAKE_MODULES_CLONE_RESULT EQUAL "0")
      message(FATAL_ERROR "failed command: git clone ${LIB_CMAKE_MODULES_URL}\n ${LIB_CMAKE_MODULES_CLONE_OUTPUT}")
    endif()

    list(APPEND CMAKE_MODULE_PATH "${CMAKE_BINARY_DIR}/cmake-modules")
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} CACHE PATH "List of directories specifying a search path for CMake modules to be loaded" FORCE)
    set(LIB_CMAKE_MODULES_EXISTS TRUE CACHE BOOL "TRUE if the cmake-modules library was cloned successfully")
  endif()
endfunction()

