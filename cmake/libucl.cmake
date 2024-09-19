include(FetchContent)

FetchContent_Declare(libucl
    GIT_REPOSITORY https://github.com/eeros-project/libucl.git
    GIT_TAG 932ff8147b6c809770e01127e4b2edbebcdf1695
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libucl-src"
    BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/libucl-build")

FetchContent_MakeAvailable(libucl)