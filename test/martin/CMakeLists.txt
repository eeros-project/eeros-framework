
#add_executable(martinTest1 MartinTest1.cpp)
#target_link_libraries(martinTest1 eeros)

if(LIBCURSES)
	add_executable(martinTest2 MartinTest2.cpp)
	target_link_libraries(martinTest2 eeros)
endif()

add_executable(martinTest3 MartinTest3.cpp)
target_link_libraries(martinTest3 eeros)

if(LIBCOMEDI)
	add_executable(martinTest4 MartinTest4.cpp)
	target_link_libraries(martinTest4 eeros)
endif()
