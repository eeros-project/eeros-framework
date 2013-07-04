#include <eeros/control/SignalBufferWriter.hpp>
#include <eeros/core/RingBuffer.hpp>
#include <eeros/control/RealSignalOutput.hpp>

SignalBufferWriter::SignalBufferWriter(void* memory, uint32_t size) : SignalBuffer(memory, size) {
	// nothing to do
}

void SignalBufferWriter::addSignal(Signal* signal) {
	observedSignals.push_back(signal);
	updateHeader();
}

void SignalBufferWriter::removeSignal(Signal* signal) {
	observedSignals.remove(signal);
	updateHeader();
}

void SignalBufferWriter::appendData() {
	RealSignalOutput* sig; // TODO change to RealSignal
	uint64_t timestamp;
	double value;
	for(std::list<Signal*>::iterator i = observedSignals.begin(); i != observedSignals.end(); i++) {
		sig = dynamic_cast<RealSignalOutput*>(*i);
		if(sig) {
			timestamp = sig->getTimestamp();
			buffer->write(&timestamp, sizeof(timestamp));
			value = sig->getValue();
			buffer->write(&value, sizeof(value));
		}
	}
}

void SignalBufferWriter::updateHeader() {
	header->version++;
	header->nofObservedSignals = 0;
	int j = 0;
	for(std::list<Signal*>::iterator i = observedSignals.begin(); i != observedSignals.end(); i++) {
		if(j < kMaxNofObservableSignals * 2) {
			header->signalInfo[j++] = (*i)->getSignalId();
			header->signalInfo[j++] = (*i)->getType();
			header->nofObservedSignals++;
		}
	}
}