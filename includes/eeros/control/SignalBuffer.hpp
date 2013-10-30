#ifndef ORG_EEROS_CONTROL_SIGNALBUFFER_HPP_
#define ORG_EEROS_CONTROL_SIGNALBUFFER_HPP_

#include <stdint.h>

namespace eeros {
	
	class RingBuffer;
	
	namespace control {

		enum { kMaxNofObservableSignals = 32 };

		enum { kSignalDataSizeReal = 16 };

		struct SignalBufferHeader {
			volatile uint32_t version;
			volatile uint32_t nofObservedSignals;
			volatile uint32_t bufferSize;
			volatile uint32_t signalInfo[2 * kMaxNofObservableSignals];
		};

		class SignalBuffer {
		public:
			SignalBuffer(void* memory, uint32_t size);
			
		protected:
			SignalBufferHeader* header;
			RingBuffer* buffer;
		};

	};
};

#endif // ORG_EEROS_CONTROL_SIGNALBUFFER_HPP_
 
 
