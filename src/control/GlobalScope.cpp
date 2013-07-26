#include <eeros/control/GlobalScope.hpp>

#include <list>
#include <stdlib.h>
#include <fcntl.h>
#include <eeros/control/Signal.hpp>
#include <eeros/control/RealSignalOutput.hpp>
#include <eeros/control/SignalBufferWriter.hpp>
#include <eeros/core/SharedMemory.hpp>

#include <iostream>
#include <sstream>

GlobalScope::GlobalScope() {
	/* create message queues */
	msqStatBuffer.mq_msgsize = kMsqMsgSize;
	msqStatBuffer.mq_maxmsg = kMsqMaxMsgs;
	inMsqDescriptor = mq_open("/eeros.csin.msq", (O_RDWR | O_CREAT | O_NONBLOCK), 0600, &msqStatBuffer);
	outMsqDescriptor = mq_open("/eeros.csout.msq", (O_RDWR | O_CREAT | O_NONBLOCK), 0600, &msqStatBuffer);
	pMsg = new char[kMsqMsgSize];
	
	/* create shared memory and signal writer */
	shm = new SharedMemory("/eeros.shm", kSharedMemorySize);
	if(shm) {
		if(!shm->initialize()) {
			writer = new SignalBufferWriter(shm->getMemoryPointer(), kSharedMemorySize);
// 			if(writer) {
// 				/* at the moment were observing all signals */
// 				std::list<Signal*>* signalList = Signal::getSignalList();
// 				for(std::list<Signal*>::iterator i = signalList->begin(); i != signalList->end(); i++) {
// 					writer->addSignal(*i);
// 				}
// 			}
		}
	}
}

GlobalScope::~GlobalScope() {
	delete writer;
	delete shm;
}

void GlobalScope::run() {
	/* check for messages */
	unsigned int msgPrio;
	int ret = mq_receive(inMsqDescriptor, pMsg, kMsqMsgSize, &msgPrio);
	if(ret >= 0) { // message received
//		std::cout << "Msg received: " << pMsg << std::endl;
		if(*pMsg == 'a' || *pMsg == 'r') {
			Signal* sig = Signal::getSignalById(*(reinterpret_cast<uint32_t*>(pMsg + 1)));
			if(sig && writer) {
				if(*pMsg == 'a') {
					std::cout << "add: " << sig->getSignalId() << std::endl;
					writer->addSignal(sig);
				}
				else if(*pMsg == 'r') {
					std::cout << "remove: " << sig->getSignalId() << std::endl;
					writer->removeSignal(sig);
				}
			}
		}
		else if(*pMsg == 'l') {
			std::list<Signal*>* signalList = Signal::getSignalList();
			for(std::list<Signal*>::iterator i = signalList->begin(); i != signalList->end(); i++) {
				std::stringstream ss;
				RealSignalOutput* realSignal = dynamic_cast<RealSignalOutput*>(*i);
				ss << realSignal->getSignalId() << '\x1D' << realSignal->getLabel() << '\x1D' <<  realSignal->getUnit() << '\x1D' << realSignal->getCoordinateSystem() << '\x1D' << realSignal->getSendingDirection();
				if(mq_send(outMsqDescriptor, ss.str().c_str(), ss.str().length() + 1, 0)) {
					std::cout << "ERROR while sending signal info...";
					break;
				}
			}
			pMsg[0] = 'e';
			pMsg[1] = 0;
			if(mq_send(outMsqDescriptor, pMsg, 2, 0)) { // send end of list
				std::cout << "ERROR while sending end of list...";
			}
		}
		else {
			// nothing to do...
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
