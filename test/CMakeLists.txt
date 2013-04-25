project(eeros-unittests)

SET(CMAKE_MODULE_PATH ${EEROS_SOURCE_DIR}/test)
find_package(cppunit)

include_directories(${EEROS_SOURCE_DIR}/includes)

add_executable(test SuiteBaseTests.cpp SignalTest.cpp)
target_link_libraries(test eeros cppunit)