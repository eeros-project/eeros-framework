#ifndef ORG_EEROS_CONTROL_SIGNALBUFFERWRITER_HPP_
#define ORG_EEROS_CONTROL_SIGNALBUFFERWRITER_HPP_

#include <list>
#include <eeros/types.hpp>
#include <eeros/control/SignalBuffer.hpp>

class Signal;

class SignalBufferWriter : public SignalBuffer {
public:
    SignalBufferWriter(void* memory, uint32_t size);
	void addSignal(sigid_t id);
	void removeSignal(sigid_t id);
	void appendData();
	
private:
	void updateHeader();
	
	std::list<sigid_t> observedSignalIds;
};

#endif // ORG_EEROS_CONTROL_SIGNALBUFFERWRITER_HPP_
 
