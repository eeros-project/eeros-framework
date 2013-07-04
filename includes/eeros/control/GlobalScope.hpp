#ifndef ORG_EEROS_CONTROL_GLOBALSCOPE_HPP
#define ORG_EEROS_CONTROL_GLOBALSCOPE_HPP

#include <list>
#include <stdint.h>
#include <eeros/core/Runnable.hpp>

//Forward Declarations
class Signal;
class SignalBufferWriter;

class GlobalScope : public Runnable {

public:
	GlobalScope(void* memory, uint32_t size);
	virtual ~GlobalScope();
	virtual void run();

private:
	SignalBufferWriter* writer;
};

#endif // ORG_EEROS_CONTROL_GLOBALSCOPE_HPP
