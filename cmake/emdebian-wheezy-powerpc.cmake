################################################################
#### CMake Toolchain file for the Emdebian Wheezy Toolchain ####
################################################################

# Platform
set(CMAKE_SYSTEM_NAME Linux)

# Compilers
set(CMAKE_C_COMPILER   powerpc-linux-gnu-gcc-4.7)
set(CMAKE_CXX_COMPILER powerpc-linux-gnu-g++-4.7)

# Target environment
set(CMAKE_FIND_ROOT_PATH /usr/powerpc-linux-gnu/)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Clear cached compiler flags
set(CMAKE_CXX_FLAGS "-std=c++0x" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "-std=c++11" CACHE STRING "" FORCE)
