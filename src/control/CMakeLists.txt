add_eeros_sources(Block.cpp Signal.cpp RealSignalInput.cpp RealSignalOutput.cpp Block1i.cpp Block1i1o.cpp Block1o.cpp BlockOutput.cpp Gain.cpp Output.cpp Input.cpp Step.cpp TimeDomain.cpp Sum.cpp D.cpp RealPeripheralInput.cpp RealPeripheralOutput.cpp Switch.cpp Constant.cpp)

# TODO remove this after implementing a version for windows...
if(LINUX)
add_eeros_sources(SignalBuffer.cpp SignalBufferWriter.cpp SignalBufferReader.cpp GlobalSignalProvider.cpp)
endif()