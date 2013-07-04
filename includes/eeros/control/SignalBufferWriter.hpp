#ifndef ORG_EEROS_CONTROL_SIGNALBUFFERWRITER_HPP_
#define ORG_EEROS_CONTROL_SIGNALBUFFERWRITER_HPP_

#include <list>
#include <eeros/types.hpp>
#include <eeros/control/SignalBuffer.hpp>

class Signal;

class SignalBufferWriter : public SignalBuffer {
public:
    SignalBufferWriter(void* memory, uint32_t size);
	void addSignal(Signal* signal);
	void removeSignal(Signal* signal);
	void appendData();
	
private:
	void updateHeader();
	
	std::list<Signal*> observedSignals;
};

#endif // ORG_EEROS_CONTROL_SIGNALBUFFERWRITER_HPP_
 
