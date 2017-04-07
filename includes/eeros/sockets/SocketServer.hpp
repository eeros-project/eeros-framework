#ifndef ORG_EEROS_SOCKET_SERVER_HPP_
#define ORG_EEROS_SOCKET_SERVER_HPP_

#include <eeros/core/Thread.hpp>
#include <atomic>
#include <array>
#include <unistd.h>
#include <netdb.h> 

// #include "../control/constants.hpp"
// #include "../control/types.hpp"

// https://vichargrave.github.io/articles/2013-02/tcp-ip-network-programming-design-patterns-in-cpp

namespace eeros {
	namespace sockets {
		
		class SocketServer : public eeros::Thread {

		public:
			
			SocketServer(uint16_t port, int byteSize = 4, double periodInSec = 0.01);
			virtual ~SocketServer();
			
			virtual void stop();
			virtual bool isRunning();
// 			virtual const std::Array& getBuffer();
// 			virtual void sendBuffer(Array data);

			int64_t readbuffer;
			
		private:
			virtual void run();
// 			Array& getWriteBuffer();
// 			Array& getSendBuffer();
// 			void flip();
			
			bool running;
			uint16_t port;
			int byteSize;
			double periodInSec;
			struct hostent *server;
			int sockfd;
			int newsockfd;
			
// 			Array read1; Array read2; Array read3;
// 			Array send1; Array send2; Array send3;
// 
// 			std::atomic<Array*> read_ptr;
// 			std::atomic<Array*> send_ptr;
		};
	};
};

#endif // ORG_EEROS_SOCKET_SERVER_HPP_