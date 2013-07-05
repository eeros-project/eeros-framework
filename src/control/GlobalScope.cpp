#include <eeros/control/GlobalScope.hpp>

#include <list>
#include <stdlib.h>
#include <eeros/control/Signal.hpp>
#include <eeros/control/SignalBufferWriter.hpp>
#include <eeros/core/SharedMemory.hpp>

GlobalScope::GlobalScope() {
	/* create shared memory and signal writer */
	shm = new SharedMemory("/eeros.shm", kSharedMemorySize);
	if(shm) {
		if(!shm->initialize()) {
			writer = new SignalBufferWriter(shm->getMemoryPointer(), kSharedMemorySize);
			if(writer) {
				/* at the moment were observing all signals */
				std::list<Signal*>* signalList = Signal::getSignalList();
				for(std::list<Signal*>::iterator i = signalList->begin(); i != signalList->end(); i++) {
					writer->addSignal(*i);
				}
			}
		}
	}
	
//	int shmInfoExchangeSize = maxNofSignals * (2 * kSignalIdSIze + kMaxLabelSize) + 2 * kLengthSize;
//	int payloadSize = shmDataExchangeSize + shmInfoExchangeSize;
//	writer = new SharedMemoryWriter(shmPath, kSharedMemoryId, shmDataExchangeSize);
}

GlobalScope::~GlobalScope() {
	delete writer;
	delete shm;
}

void GlobalScope::run() {
	if(writer) {
		writer->appendData();
	}
}

void* GlobalScope::getSharedMemory() {
	if(shm) return shm->getMemoryPointer();
	return 0;
}