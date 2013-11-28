project(eeros-unittests)

SET(CMAKE_MODULE_PATH ${EEROS_SOURCE_DIR}/test)
find_package(cppunit)

include_directories(${EEROS_SOURCE_DIR}/includes ${EEROS_BINARY_DIR})

#add_executable(test SuiteBaseTests.cpp SignalTest.cpp)
#target_link_libraries(test cppunit eeros )

add_executable(cppUnitTest SequenceMoveException8b.cpp SequenceMoveException8a.cpp NonBlockingSubSequence_ErrorHandler.cpp SequenceMoveException6b.cpp SequenceMoveException6a.cpp 
               CallingNonBlockingSequence_ErrorHandler.cpp SequenceMoveException2b.cpp SequenceMoveException2a.cpp ErrorHandlerB.cpp ErrorHandlerA.cpp 
               SequenceMoveException1_3.cpp NonBlockingSubSequence.cpp MySequencer.cpp  
               CallingNonBlockingSubSequence.cpp CPPUnitTestMain.cpp SequencerTest.cpp MySequence.cpp CallingSubSequence.cpp BlockingSubSequence.cpp)
target_link_libraries(cppUnitTest cppunit eeros )

add_executable(martinTest1 MartinTest1.cpp)
target_link_libraries(martinTest1 eeros)

add_executable(martinTest2 MartinTest2.cpp)
target_link_libraries(martinTest2 eeros)

add_executable(ringBufferTest RingBufferTest.cpp)
target_link_libraries(ringBufferTest eeros)

add_executable(ringBufferProcessTest RingBufferProcessTest.cpp)
target_link_libraries(ringBufferProcessTest eeros)

if(NOT WINDOWS)
	add_executable(martinTest3 MartinTest3.cpp)
	target_link_libraries(martinTest3 eeros)
	
	add_executable(martinTest4 MartinTest4.cpp)
	target_link_libraries(martinTest4 eeros)
	
#	add_executable(sharedMemoryReaderTest SharedMemoryReaderTest.cpp)
#	target_link_libraries(sharedMemoryReaderTest eeros)

	add_executable(vinciTest1 VinciTest1.cpp)
	target_link_libraries(vinciTest1 eeros)
endif()