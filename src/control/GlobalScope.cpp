#include <eeros/control/GlobalScope.hpp>

#include <list>
#include <stdlib.h>
#include <fcntl.h>
#include <eeros/control/Signal.hpp>
#include <eeros/control/SignalBufferWriter.hpp>
#include <eeros/core/SharedMemory.hpp>

#include <iostream>

GlobalScope::GlobalScope() {
	/* create message queue */
	msqStatBuffer.mq_msgsize = kMsqMsgSize;
	msqStatBuffer.mq_maxmsg = kMsqMaxMsgs;
	msqDescriptor = mq_open("/eeros.msq", (O_RDWR | O_CREAT), 0600, &msqStatBuffer);
	pMsg = new char[kMsqMsgSize];
	
	/* create shared memory and signal writer */
	shm = new SharedMemory("/eeros.shm", kSharedMemorySize);
	if(shm) {
		if(!shm->initialize()) {
			writer = new SignalBufferWriter(shm->getMemoryPointer(), kSharedMemorySize);
//			if(writer) {
//				/* at the moment were observing all signals */
// 				std::list<Signal*>* signalList = Signal::getSignalList();
// 				for(std::list<Signal*>::iterator i = signalList->begin(); i != signalList->end(); i++) {
// 					writer->addSignal(*i);
// 				}
//			}
		}
	}
}

GlobalScope::~GlobalScope() {
	delete writer;
	delete shm;
}

void GlobalScope::run() {
	/* check for messages */
	unsigned int msgPrio = 0;
	int ret = mq_receive(msqDescriptor, pMsg, kMsqMsgSize, &msgPrio);
	if(ret >= 0) { // message received
		//std::cout << "Msg: " << (char)(*pMsg) << (int)(*(pMsg+1)) << std::endl;
		Signal* sig = Signal::getSignalById(*(reinterpret_cast<uint32_t*>(pMsg + 1)));
		if(sig && writer) {
			if(*pMsg == 'a') {
				writer->addSignal(sig);
			}
			else if(*pMsg == 'r') {
				writer->removeSignal(sig);
			}
		}
	}
	
	/* copy observed signals to shared memory */
	if(writer) {
		writer->appendData();
	}
}

void* GlobalScope::getSharedMemory() {
	if(shm) return shm->getMemoryPointer();
	return 0;
}