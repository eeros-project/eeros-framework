include(FetchContent)

FetchContent_Declare(libucl
    GIT_REPOSITORY https://github.com/eeros-project/libucl.git
    GIT_TAG master
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libucl-src"
    BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/libucl-build")

FetchContent_MakeAvailable(libucl)
eeros_add_package(eeros ucl)
