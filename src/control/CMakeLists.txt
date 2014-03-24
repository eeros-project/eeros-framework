add_eeros_sources(Block.cpp TimeDomain.cpp)

# TODO remove this after implementing a version for windows...
if(LINUX)
add_eeros_sources(SignalBuffer.cpp SignalBufferWriter.cpp SignalBufferReader.cpp GlobalSignalProvider.cpp)
endif()