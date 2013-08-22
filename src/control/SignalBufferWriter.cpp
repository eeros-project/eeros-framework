#include <eeros/control/SignalBufferWriter.hpp>
#include <eeros/core/RingBuffer.hpp>
#include <eeros/control/RealSignalOutput.hpp>

#include <iostream>

SignalBufferWriter::SignalBufferWriter(void* memory, uint32_t size) : SignalBuffer(memory, size) {
	// nothing to do
}

void SignalBufferWriter::addSignal(sigid_t id) {
	observedSignalIds.push_back(id);
	// Remove duplicates
	observedSignalIds.unique();
	updateHeader();
}

void SignalBufferWriter::removeSignal(sigid_t id) {
	observedSignalIds.remove(id);
	updateHeader();
}

void SignalBufferWriter::appendData() {
	for(std::list<sigid_t>::iterator i = observedSignalIds.begin(); i != observedSignalIds.end(); i++) {
		Signal* signal = Signal::getSignalById(*i);
		RealSignalOutput* realSignalOutput = dynamic_cast<RealSignalOutput*>(signal);
		if(realSignalOutput) {
			uint64_t timestamp = realSignalOutput->getTimestamp((sigindex_t)*i);
			buffer->write(&timestamp, sizeof(timestamp));
			double value = realSignalOutput->getValue((sigindex_t)*i);
			buffer->write(&value, sizeof(value));
		}
	}
}

void SignalBufferWriter::updateHeader() { // TODO implement with mutex in shm
	header->nofObservedSignals = 0;
	uint32_t signalCounter = 0;
	int currentIndex = 0;
	for(std::list<sigid_t>::iterator i = observedSignalIds.begin(); i != observedSignalIds.end(); i++) {
		if(currentIndex < kMaxNofObservableSignals * 2) {
			Signal* signal = Signal::getSignalById(*i);
			if (signal == NULL) {
				throw 13;
			}
			sigid_t id = signal->getSignalId((sigindex_t)*i);
			header->signalInfo[currentIndex++] = id;
			header->signalInfo[currentIndex++] = signal->getType();
			signalCounter++;
		}
	}
	header->nofObservedSignals = signalCounter;
	header->version++;
}
