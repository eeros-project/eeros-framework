project(eeros-unittests)

SET(CMAKE_MODULE_PATH ${EEROS_SOURCE_DIR}/test)
find_package(cppunit)

include_directories(${EEROS_SOURCE_DIR}/includes ${EEROS_BINARY_DIR})

#add_executable(test SuiteBaseTests.cpp SignalTest.cpp)
#target_link_libraries(test cppunit eeros )

add_executable(martinTest1 MartinTest1.cpp)
target_link_libraries(martinTest1 eeros)

add_executable(martinTest2 MartinTest2.cpp)
target_link_libraries(martinTest2 eeros)

if(NOT WINDOWS)
	add_executable(martinTest3 MartinTest3.cpp)
	target_link_libraries(martinTest3 eeros)
endif()