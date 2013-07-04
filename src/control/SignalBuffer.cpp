#include <eeros/control/SignalBuffer.hpp>
#include <eeros/core/RingBuffer.hpp>

SignalBuffer::SignalBuffer(void* memory, uint32_t size) {;
	uint32_t ringBufferSize = size - sizeof(SignalBufferHeader); // TODO optimize size (power of two)
	void* pRingBuffer =  static_cast<char*>(memory) + sizeof(SignalBufferHeader);
	
	header = static_cast<SignalBufferHeader*>(memory);
 	buffer = new RingBuffer(pRingBuffer, ringBufferSize);
	
// 	header->version = 0;
// 	header->nofObservedSignals = 0;
	header->bufferSize = ringBufferSize;
}
