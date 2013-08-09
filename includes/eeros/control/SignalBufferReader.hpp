#ifndef ORG_EEROS_CONTROL_SIGNALBUFFERREADER_HPP_
#define ORG_EEROS_CONTROL_SIGNALBUFFERREADER_HPP_

#include <stdint.h>
#include <list>
#include <eeros/types.hpp>
#include <eeros/control/SignalBuffer.hpp>

class Signal;
class RealSignalOutput;

struct SignalInfo {
	SignalInfo(sigid_t id, sigtype_t type) : id(id), type(type) { }
	sigid_t id;
	sigtype_t type;
};

class SignalBufferReader : public SignalBuffer {
public:
	SignalBufferReader(void* memory, uint32_t size);
	uint32_t nofObservedSignals();
	uint32_t signalTypeAvailableToRead();
	bool readRealSignal(sigid_t* id, uint64_t* timestamp, double* value);
	
private:
	sigtype_t getSignalType(int readIndex);
	sigid_t getSignalId(int readIndex);
	uint32_t getSignalDataSize(sigtype_t signalType);
	void correctHeaderVersion();
	void incrementReadIndex();
	
	int readIndex;
	uint8_t headerVersion;
};

#endif // ORG_EEROS_CONTROL_SIGNALBUFFERREADER_HPP_
