#include <eeros/control/GlobalScope.hpp>
#include <eeros/control/Signal.hpp>
#include <eeros/control/SignalBufferWriter.hpp>
#include <list>
#include <stdlib.h>

enum {
	kSharedMemorySize = 1048576, // 1MB
	kAnSignalValueSize = 8,
	kSignalTimestampSize = 8,
};

// struct realSignalDatum {
// 	realSignalDatum(uint32_t signalId, uint64_t timestamp, double value) : signalId(signalId), timestamp(timestamp), value(value) {}
// 	uint32_t signalId;
// 	uint64_t timestamp;
// 	double value;
// };

GlobalScope::GlobalScope(void* memory, uint32_t size) {
	//void* memory = malloc(kSharedMemorySize);
	writer = new SignalBufferWriter(memory, size);
	
	std::list<Signal*>* signalList = Signal::getSignalList();
	for(std::list<Signal*>::iterator i = signalList->begin(); i != signalList->end(); i++) {
		writer->addSignal(*i);
	}
	
//	int shmInfoExchangeSize = maxNofSignals * (2 * kSignalIdSIze + kMaxLabelSize) + 2 * kLengthSize;
//	int payloadSize = shmDataExchangeSize + shmInfoExchangeSize;
//	writer = new SharedMemoryWriter(shmPath, kSharedMemoryId, shmDataExchangeSize);
}

GlobalScope::~GlobalScope() {
	delete writer;
}

void GlobalScope::run() {
	if(writer) {
		writer->appendData();
		
// 		std::list<Signal*>* signalList = Signal::getSignalList();
// 		RealSignalOutput* sig;
// 		realSignalDatum* sigDat;
// 		for(std::list<Signal*>::iterator i = signalList->begin(); i != signalList->end(); i++) {
// 			sig = dynamic_cast<RealSignalOutput*>(*i);
// 			if(sig) {
// 				sigDat = new realSignalDatum(sig->getSignalId(), sig->getTimestamp(), sig->getValue());
// 				writer->write<realSignalDatum>(sigDat);
// 			}
// 		}
	}
}