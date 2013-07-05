#ifndef ORG_EEROS_CONTROL_GLOBALSCOPE_HPP
#define ORG_EEROS_CONTROL_GLOBALSCOPE_HPP

#include <list>
#include <stdint.h>
#include <eeros/core/Runnable.hpp>

//Forward Declarations
class Signal;
class SignalBufferWriter;
class SharedMemory;

enum {
	kSharedMemorySize = 1048576, // 1MB
};

class GlobalScope : public Runnable {

public:
	GlobalScope();
	virtual ~GlobalScope();
	virtual void run();
	
	virtual void* getSharedMemory();

private:
	SignalBufferWriter* writer;
	SharedMemory* shm;
};

#endif // ORG_EEROS_CONTROL_GLOBALSCOPE_HPP
