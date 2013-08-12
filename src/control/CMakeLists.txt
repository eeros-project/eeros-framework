add_eeros_sources(Block.cpp Signal.cpp RealSignalInput.cpp RealSignalOutput.cpp Block1i.cpp Block1i1o.cpp Block1i3o.cpp Block1o.cpp Block2i1o.cpp BlockOutput.cpp Gain.cpp Output.cpp PIControl.cpp Input.cpp Step.cpp TimeDomain.cpp Sum.cpp)

# TODO remove this after implementing a version for windows...
if(LINUX)
add_eeros_sources(SignalBuffer.cpp SignalBufferWriter.cpp SignalBufferReader.cpp GlobalScope.cpp)
endif()