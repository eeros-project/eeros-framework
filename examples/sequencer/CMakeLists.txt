# if(LIBCURSES)
# 	add_executable(simpleSequencerExample simpleExample.cpp)
# 	target_link_libraries(simpleSequencerExample eeros)
# endif()
add_executable(sequencerTest1 SequencerTest1.cpp)
target_link_libraries(sequencerTest1 eeros ${EEROS_LIBS})

add_executable(sequencerTest2 SequencerTest2.cpp)
target_link_libraries(sequencerTest2 eeros ${EEROS_LIBS})

add_executable(sequencerTest3 SequencerTest3.cpp)
target_link_libraries(sequencerTest3 eeros ${EEROS_LIBS})

add_executable(sequencerTest4 SequencerTest4.cpp)
target_link_libraries(sequencerTest4 eeros ${EEROS_LIBS})

add_executable(sequencerTest5 SequencerTest5.cpp)
target_link_libraries(sequencerTest5 eeros ${EEROS_LIBS})
