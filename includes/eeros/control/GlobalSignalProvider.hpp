#ifndef ORG_EEROS_CONTROL_GLOBALSIGNALPROVIDER_HPP
#define ORG_EEROS_CONTROL_GLOBALSIGNALPROVIDER_HPP

#include <list>
#include <stdint.h>
#include <mqueue.h>
#include <sstream>
#include <eeros/core/Runnable.hpp>

namespace eeros {
	
	//Forward Declarations
	class SharedMemory;
	
	namespace control {

		//Forward Declarations
		class SignalBufferWriter;
		
		enum { 
			kMsqMsgSize = 1024,
			kMsqMaxMsgs = 10 // has to be smaller or equal to /proc/sys/fs/mqueue/msg_max
			// IMPORTANT: kMsqMsgSize * kMsqMaxMsgs must not be bigger than RLIMIT_MSGQUEUE.
			// You can get RLIMIT_MSGQUEUE with command "ulimit -q".
		};

		enum { kSharedMemorySize = 1048576 * 10}; // 10 MB

		class GlobalSignalProvider : public Runnable {

		public:
			GlobalSignalProvider();
			virtual ~GlobalSignalProvider();
			virtual void run();
			
			virtual void* getSharedMemory();

		private:
			void processIPCMessages();
			void sendIPCMessage(std::stringstream& ss);

		private:
			mqd_t inMsqDescriptor;
			mqd_t outMsqDescriptor;
			struct mq_attr msqStatBuffer;
			char* pMsg;
			
			SignalBufferWriter* writer;
			SharedMemory* shm;
		};

	};
};
#endif // ORG_EEROS_CONTROL_GLOBALSIGNALPROVIDER_HPP
