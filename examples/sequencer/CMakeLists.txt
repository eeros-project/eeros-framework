add_executable(sequencer MySequence.cpp MyBlockingSubSequence.cpp MySequencer.cpp Main.cpp MyNonBlockingSubSequence.cpp MySubSequencer.cpp)
target_link_libraries(sequencer eeros )
