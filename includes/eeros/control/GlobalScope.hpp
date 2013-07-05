#ifndef ORG_EEROS_CONTROL_GLOBALSCOPE_HPP
#define ORG_EEROS_CONTROL_GLOBALSCOPE_HPP

#include <list>
#include <stdint.h>
#include <mqueue.h>
#include <eeros/core/Runnable.hpp>

//Forward Declarations
class Signal;
class SignalBufferWriter;
class SharedMemory;

enum { 
	kMsqMsgSize = 1024, 
	kMsqMaxMsgs = 10 // has to be smaller or equal to /proc/sys/fs/mqueue/msg_max
};

enum { kSharedMemorySize = 1048576 }; // 1 MB

class GlobalScope : public Runnable {

public:
	GlobalScope();
	virtual ~GlobalScope();
	virtual void run();
	
	virtual void* getSharedMemory();

private:
	mqd_t msqDescriptor;
	struct mq_attr msqStatBuffer;
	char* pMsg;
	
	SignalBufferWriter* writer;
	SharedMemory* shm;
};

#endif // ORG_EEROS_CONTROL_GLOBALSCOPE_HPP
