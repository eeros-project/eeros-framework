#
#  CMake Modules Library Loader
# 
#
#  This module provides a load function to clone the CMake modules library into the project using git.
#  Since this modules typically exists as a copy in the target projects, it is equipped
#  with a version number using Semantic Versioning convention.
#  When improving this module, please feed the changes back to the upstream project
#  by a pull request to https://github.com/andreaskunz/cmake-modules  Thanks!
#
#
#  Copyright 2021 Andreas Kunz
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# 
#
#  v0.1.1
#


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

  if(${ARGC} EQUAL 1 AND ARGV0)
    set(LIB_CMAKE_MODULES_URL ${ARGV0} CACHE STRING "The path to the origin of the cmake-modules library")
  endif()

  if(NOT LIB_CMAKE_MODULES_CLONED)
    set(LIB_CMAKE_MODULES_URL https://github.com/andreaskunz/cmake-modules.git CACHE STRING "The path to the origin of the cmake-modules library")

    execute_process(COMMAND ${GIT_EXECUTABLE} clone ${LIB_CMAKE_MODULES_URL}
                    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                    RESULT_VARIABLE LIB_CMAKE_MODULES_CLONE_RESULT
                    ERROR_VARIABLE LIB_CMAKE_MODULES_CLONE_OUTPUT)
    if(NOT LIB_CMAKE_MODULES_CLONE_RESULT EQUAL "0")
      message(FATAL_ERROR "failed command: git clone ${LIB_CMAKE_MODULES_URL}\n ${LIB_CMAKE_MODULES_CLONE_OUTPUT}")
    endif()

    set(LIB_CMAKE_MODULES_ROOT_PATH "${CMAKE_BINARY_DIR}/cmake-modules" CACHE PATH "Root directory path of the CMake modules library" FORCE)
    list(APPEND CMAKE_MODULE_PATH ${LIB_CMAKE_MODULES_ROOT_PATH})
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} CACHE PATH "List of directories specifying a search path for CMake modules to be loaded" FORCE)
    set(LIB_CMAKE_MODULES_CLONED TRUE CACHE BOOL "TRUE if the cmake-modules library was cloned successfully")
  endif()
endfunction()

