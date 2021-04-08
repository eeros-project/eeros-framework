#ifndef ORG_EEROS_SOCKET_SERVER_HPP_
#define ORG_EEROS_SOCKET_SERVER_HPP_

#include <eeros/core/Thread.hpp>
#include <atomic>
#include <array>
#include <unistd.h>
#include <netdb.h> 
#include <arpa/inet.h> 		/* inet_ntoa() to format IP address */
#include <string.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <eeros/core/Fault.hpp>
#include <iostream>
#include <cstring>
#include <signal.h>

namespace eeros {
	namespace sockets {
				
		void sigPipeHandler(int signum);

		template < uint32_t BufInLen, typename inT, uint32_t BufOutLen, typename outT >
		class SocketServer : public eeros::Thread {
		public:
			SocketServer(uint16_t port, double period = 0.01, double timeout = 1.0, int priority = 5) : Thread(priority), read1({0}), read2({0}), read3({0}) {
				this->port = port;
				this->period = period;
				this->timeout = timeout;
				signal(SIGPIPE, sigPipeHandler);	// make sure, that a broken pipe does not stop application
				read_ptr.store(&read1);
				send_ptr.store(&send1);
				running = false;
				connected = false;
			}
			
			virtual ~SocketServer() {
				join();
			}
			
			virtual void stop() {
				running = false;
			}
			
			virtual bool isRunning() {
				return running;
			}
			
			virtual bool isConnected() {
				return connected;
			}
			
			virtual std::array<outT, BufOutLen>& getReceiveBuffer() {
				return *read_ptr.load();
			}
			
			virtual void setSendBuffer(std::array<inT, BufInLen>& data) {
				auto p = send_ptr.load();
				if (p == &send1) send1 = data;
				else if (p == &send2) send2 = data;
				else if (p == &send3) send3 = data;
			}

			outT readbuffer;
			bool newData = false;
			
		private:
			virtual void run() {	
				log.info() << "SocketServer thread started";
				struct sockaddr_in servAddr;
				sockfd = socket(AF_INET, SOCK_STREAM, 0);
				if (sockfd < 0) throw Fault("ERROR opening socket");
				
				bzero((char *) &servAddr, sizeof(servAddr));	
				servAddr.sin_port = htons(port); 
				servAddr.sin_family = AF_INET;
				servAddr.sin_addr.s_addr = htonl(INADDR_ANY) ;
				int yes = 1;
				if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1)
					throw Fault("ERROR on set socket option");
				if (bind(sockfd, (struct sockaddr *) &servAddr, sizeof(servAddr)) < 0) 
					throw Fault("ERROR on socket binding");
				
				socklen_t clilen;
				listen(sockfd,1);
				struct sockaddr_in cliAddr;
				clilen = sizeof(cliAddr);
				
				running = true;
				while (running) {
					newsockfd = accept(sockfd, (struct sockaddr *) &cliAddr,  &clilen);
					if (newsockfd < 0) throw Fault("ERROR on socket accept");
					connected = true;
					char cliName[INET6_ADDRSTRLEN];
					getnameinfo((struct sockaddr*)&cliAddr, sizeof cliAddr, cliName, sizeof(cliName), NULL, 0, NI_NUMERICHOST|NI_NUMERICSERV);
					log.info() << "Client connection from ip=" << cliName << " accepted";
					inT b_write[BufInLen]; outT b_read[BufOutLen];
					using seconds = std::chrono::duration<double, std::chrono::seconds::period>;
					auto next_cycle = std::chrono::steady_clock::now() + seconds(period);
				
					while (connected) {
						std::this_thread::sleep_until(next_cycle);
			
						// write
						std::array<inT, BufInLen> &sendValue = getNextSendBuffer();
						for(uint32_t i = 0; i < BufInLen; i++) b_write[i] = sendValue[i]; 
// 						log.trace() << "try to write " << b_write[0];
						int n = write(newsockfd, b_write, BufInLen * sizeof(inT));
						if (n < 0) {
							log.trace() << "error = " << std::strerror(errno);
							connected = false;
						}	
						
						// read
						size_t count = BufOutLen * sizeof(outT);
						uint8_t* ptr = (uint8_t *)b_read;
						auto endTime = std::chrono::steady_clock::now() + seconds(timeout);
						while (connected && count) {
							if (std::chrono::steady_clock::now() > endTime) {
								log.trace() << "error = socket read timed out";
								connected = false;
							}
// 							log.trace() << "try to read " << count << " Bytes";
							n = read(newsockfd, ptr, count);
							if (n < 0) {
								log.trace() << "error = " << std::strerror(errno);
								connected = false;
							}
							ptr += n;
							count -= n;
						}
						std::array<outT, BufOutLen> &readValue = getNextReceiveBuffer();
						for(uint32_t i = 0; i < BufOutLen; i++) readValue[i] = b_read[i];
						newData = true;
						flip();
						next_cycle += seconds(period);
					}
					close(newsockfd);
					// if disconnected clear receive buffer
					std::array<outT, BufOutLen> &readValue = getNextReceiveBuffer();
					for(uint32_t i = 0; i < BufOutLen; i++) readValue[i] = 0;
					newData = true;
					flip();
				}
				close(sockfd);
			}
			
			std::array<outT, BufOutLen>& getNextReceiveBuffer() {
				auto p = read_ptr.load();
				if (p == &read1) return read2;
				else if (p == &read2) return read3;
				else return read1;
			}
			
			std::array<inT, BufInLen>& getNextSendBuffer() {
				auto p = send_ptr.load();
				if (p == &send1) return send2;
				else if (p == &send2) return send3;
				else return send1;
			}
			
			void flip(){
				read_ptr.store(&getNextReceiveBuffer());
				send_ptr.store(&getNextSendBuffer());
			}
			
			bool running;
			uint16_t port;
			double period;
			double timeout;	// time which thread tries to read until socket read timed out
			struct hostent *server;
			int sockfd;
			int newsockfd;
			bool connected;
			
			std::array<outT, BufOutLen> read1, read2, read3;
			std::array<inT, BufInLen> send1, send2, send3;
			std::atomic< std::array<outT, BufOutLen>* > read_ptr;
			std::atomic< std::array<inT, BufInLen>* > send_ptr;
		};

		// specialization used when server doesn't receive data from its client
		template < uint32_t BufInLen, typename inT >
		class SocketServer<BufInLen, inT, 0, std::nullptr_t> : public eeros::Thread {
		public:
			SocketServer(uint16_t port, double period = 0.01, double timeout = 1.0, int priority = 5) : Thread(priority) {
				this->port = port;
				this->period = period;
				signal(SIGPIPE, sigPipeHandler);	// make sure, that a broken pipe does not stop application
				send_ptr.store(&send1);
				running = false;
				connected = false;
			}
			
			virtual ~SocketServer() {
				join();
			}
			
			virtual void stop(){
				running = false;
			}
			
			virtual bool isRunning() {
				return running;
			}
			
			virtual bool isConnected() {
				return connected;
			}
			
			virtual void setSendBuffer(std::array<inT, BufInLen>& data) {
				auto p = send_ptr.load();
				if (p == &send1) send1 = data;
				else if (p == &send2) send2 = data;
				else if (p == &send3) send3 = data;
			}

		private:
			virtual void run() {	
				log.info() << "SocketServer thread started";
				struct sockaddr_in servAddr;
				sockfd = socket(AF_INET, SOCK_STREAM, 0);
				if (sockfd < 0) throw Fault("ERROR opening socket");
				
				bzero((char *) &servAddr, sizeof(servAddr));	
				servAddr.sin_port = htons(port); 
				servAddr.sin_family = AF_INET;
				servAddr.sin_addr.s_addr = htonl(INADDR_ANY) ;
				int yes = 1;
				if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1)
					throw Fault("ERROR on set socket option");
				if (bind(sockfd, (struct sockaddr *) &servAddr, sizeof(servAddr)) < 0) 
					throw Fault("ERROR on socket binding");
				
				socklen_t clilen;
				listen(sockfd,1);
				struct sockaddr_in cliAddr;
				clilen = sizeof(cliAddr);
				
				running = true;
				while (running) {
					newsockfd = accept(sockfd, (struct sockaddr *) &cliAddr,  &clilen);
					if (newsockfd < 0) throw Fault("ERROR on accept");
					connected = true;
					char cliName[INET6_ADDRSTRLEN];
					getnameinfo((struct sockaddr*)&cliAddr, sizeof cliAddr, cliName, sizeof(cliName), NULL, 0, NI_NUMERICHOST|NI_NUMERICSERV);
					log.info() << "Client connection from ip=" << cliName << " accepted";
					inT b_write[BufInLen];
					using seconds = std::chrono::duration<double, std::chrono::seconds::period>;
					auto next_cycle = std::chrono::steady_clock::now() + seconds(period);
				
					while (connected) {
						std::this_thread::sleep_until(next_cycle);
					
						// write
						std::array<inT, BufInLen> &sendValue = getNextSendBuffer();
						for(uint32_t i = 0; i < BufInLen; i++) b_write[i] = sendValue[i]; 
// 						log.trace() << "try to write " << b_write[0];
						int n = write(newsockfd, b_write, BufInLen * sizeof(inT));
						if (n < 0) {
							log.trace() << "error = " << std::strerror(errno);
							connected = false;
						}	
				
						flip();
						next_cycle += seconds(period);
					}
					close(newsockfd);
				}
				close(sockfd);
			}
			
			std::array<inT, BufInLen>& getNextSendBuffer() {
				auto p = send_ptr.load();
				if (p == &send1) return send2;
				else if (p == &send2) return send3;
				else return send1;
			}
			
			void flip(){
				send_ptr.store(&getNextSendBuffer());
			}
			
			bool running;
			uint16_t port;
			double period;
			struct hostent *server;
			int sockfd;
			int newsockfd;
			bool connected;
			
			std::array<inT, BufInLen> send1, send2, send3;
			std::atomic< std::array<inT, BufInLen>* > send_ptr;
		};

	};
};

#endif // ORG_EEROS_SOCKET_SERVER_HPP_
