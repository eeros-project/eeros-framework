#include <eeros/control/GlobalSignalProvider.hpp>

#include <list>
#include <stdlib.h>
#include <fcntl.h>
#include <eeros/control/Signal.hpp>
#include <eeros/control/RealSignalOutput.hpp>
#include <eeros/control/SignalBufferWriter.hpp>
#include <eeros/core/SharedMemory.hpp>
#include <eeros/core/System.hpp>

#include <iostream>
#include <cerrno>

GlobalSignalProvider::GlobalSignalProvider() {
	/* create message queues */
	msqStatBuffer.mq_msgsize = kMsqMsgSize;
	msqStatBuffer.mq_maxmsg = kMsqMaxMsgs;
	inMsqDescriptor = mq_open("/eeros.csin.msq", (O_RDWR | O_CREAT | O_NONBLOCK), 0600, &msqStatBuffer);
	if (inMsqDescriptor == -1) {
		std::cout << "ERROR in GlobalSignalProvider while opening inMsqDescriptor" << std::endl;
	}
	outMsqDescriptor = mq_open("/eeros.csout.msq", (O_RDWR | O_CREAT | O_NONBLOCK), 0600, &msqStatBuffer);
	if (outMsqDescriptor == -1) {
		std::cout << "ERROR in GlobalSignalProvider while opening outMsqDescriptor" << std::endl;
	}
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

GlobalSignalProvider::~GlobalSignalProvider() {
	delete writer;
	delete shm;
	delete pMsg;
}

void GlobalSignalProvider::run() {
	/* check for messages */
	uint32_t msgPrio;
	int32_t ret = mq_receive(inMsqDescriptor, pMsg, kMsqMsgSize, &msgPrio);
	if (ret > 0) {
		processIPCMessages();
	}
	/* copy observed signals to shared memory */
	if (writer) {
		writer->appendData();
	}
}

void GlobalSignalProvider::processIPCMessages() {
	switch(pMsg[0]) {
		case 'a': // Fall trough
		case 'r': {
			sigid_t signalId = *(reinterpret_cast<sigid_t*>(pMsg + 1));
			if (signalId != 0 && writer) {
				if (pMsg[0] == 'a') {
					std::cout << "add: " << signalId << std::endl;
					writer->addSignal(signalId);
				} else if (pMsg[0] == 'r') {
					std::cout << "remove: " << signalId << std::endl;
					writer->removeSignal(signalId);
				}
			}
			break;
		}
		case 'l': {
			std::list<Signal*>* signalList = Signal::getSignalList();
			int32_t collectedSignals = 0;
			std::stringstream ss;
			for (std::list<Signal*>::iterator it = signalList->begin(); it != signalList->end(); it++) {
				RealSignalOutput* realSignal = dynamic_cast<RealSignalOutput*>(*it);
				for (sigindex_t i = 0; i < realSignal->getDimension(); i++) {
					// Collect signals to reduce the number of IPC messages and send them batched
					ss << realSignal->getSignalId(i) << '\x1D' << realSignal->getLabel(i) << '\x1D' << realSignal->getSendingDirection(i) << '\x1E';
					collectedSignals++;
					const int32_t NUMBER_OF_SIGNAL_TO_COLLECT = 10;
					if (collectedSignals >= NUMBER_OF_SIGNAL_TO_COLLECT) {
						collectedSignals = 0;
						sendIPCMessage(ss);
					}
				}
			}
			// Check if there are still some collected and not yet sended signals available
			if (collectedSignals >= 1) {
				collectedSignals = 0;
				sendIPCMessage(ss);
			}
			pMsg[0] = 'e';
			pMsg[1] = 0;
			if (mq_send(outMsqDescriptor, pMsg, 2, 0)) { // send end of list
				std::cout << "ERROR while sending end of list..." << std::endl;
			}
			break;
		}
		case 's': {
			int32_t index = 1;
			int32_t nofSignals = *(reinterpret_cast<int32_t*>(pMsg + index));
			index += sizeof(int32_t);
			for (int32_t i = 0; i < nofSignals; i++) {
				uint32_t signalId = *(reinterpret_cast<uint32_t*>(pMsg + index));
				index += sizeof(signalId);

				// We just ignore the received timestamp
				uint64_t timestamp = System::getTimeNs();
				index += sizeof(timestamp);

				double value = *(reinterpret_cast<double*>(pMsg + index));
				index += sizeof(double);
				Signal* signal = Signal::getSignalById(signalId);
				RealSignalOutput* realSignal = dynamic_cast<RealSignalOutput*>(signal);
				realSignal->setValue(value, (sigindex_t) signalId);
				realSignal->setTimeStamp(timestamp, (sigindex_t) signalId);
			}
			break;
		}
		default:
			// Nothing to do...
			break;
	}
}

void GlobalSignalProvider::sendIPCMessage(std::stringstream& ss) {
	int32_t tryCounter = 0;
	const int32_t MAX_TRIES = 1;
	while (tryCounter < MAX_TRIES) {
		if (mq_send(outMsqDescriptor, ss.str().c_str(), ss.str().length() + 1, 0)) {
			if (errno == EAGAIN) {
				tryCounter++;
			} else {
				std::cout << "ERROR while sending signal info. errno: " << errno << std::endl;
				break;
			}
		} else {
			break;
		}
	}
	if (tryCounter >= MAX_TRIES) {
		std::cout << "GlobalSignalProvider::sendIPCMessage(): Max number of tries reached." << std::endl;
	}
	// Clear the stream
	ss.str(std::string());
}

void* GlobalSignalProvider::getSharedMemory() {
	if(shm) return shm->getMemoryPointer();
	return 0;
}
