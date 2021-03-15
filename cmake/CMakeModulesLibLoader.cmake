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


  if(NOT CMAKE_MODULES_LIB_EXISTS)
    set(CMAKE_MODULES_LIB_URL https://github.com/andreaskunz/cmake-modules.git CACHE STRING "The path to the origin of the cmake-modules library")

    execute_process(COMMAND ${GIT_EXECUTABLE} clone ${CMAKE_MODULES_LIB_URL}
                    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                    RESULT_VARIABLE CMAKE_MODULES_LIB_CLONE_RESULT
                    ERROR_VARIABLE CMAKE_MODULES_LIB_CLONE_OUTPUT)
    if(NOT CMAKE_MODULES_LIB_CLONE_RESULT EQUAL "0")
      message(FATAL_ERROR "failed command: git clone ${CMAKE_MODULES_LIB_URL}\n ${CMAKE_MODULES_LIB_CLONE_OUTPUT}")
    endif()

    set(CMAKE_MODULES_LIB_PATH "${CMAKE_BINARY_DIR}/cmake-modules" CACHE PATH "The path to the cmake-modules library")
    set(CMAKE_MODULES_LIB_EXISTS TRUE CACHE BOOL "TRUE if the cmake-modules library was cloned successfully")
  endif()
endfunction()

