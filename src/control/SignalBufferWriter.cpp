#include <eeros/control/SignalBufferWriter.hpp>
#include <eeros/control/Signal.hpp>
#include <eeros/core/RingBuffer.hpp>

#include <iostream>

using namespace eeros::control;

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
	for(auto id : observedSignalIds) {
		Signal<double>* doubleSignal = dynamic_cast<Signal<double>*>(Signal<double>::getSignalById(id));
		if(doubleSignal != nullptr) {
			uint64_t timestamp = doubleSignal->getTimestamp();
			buffer->write(&timestamp, sizeof(timestamp));
			double value = doubleSignal->getValue();
			buffer->write(&value, sizeof(value));
		}
	}
}

void SignalBufferWriter::updateHeader() { // TODO implement with mutex in shm
	header->nofObservedSignals = 0;
	uint32_t signalCounter = 0;
	int currentIndex = 0;
	for(auto id : observedSignalIds) {
		if(currentIndex < kMaxNofObservableSignals * 2) {
			SignalInterface* signal = Signal<double>::getSignalById(id);
			if(signal == nullptr) {
				throw "SignalBufferWriter::updateHeader(): Signal not found";
			}
			header->signalInfo[currentIndex++] = id;
			header->signalInfo[currentIndex++] = kSignalTypeReal;
			signalCounter++;
		}
	}
	header->nofObservedSignals = signalCounter;
	header->version++;
}
