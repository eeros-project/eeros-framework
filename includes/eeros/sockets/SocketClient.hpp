#ifndef ORG_EEROS_SOCKET_CLIENT_HPP_
#define ORG_EEROS_SOCKET_CLIENT_HPP_

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
		
		template < uint32_t BufInLen, typename inT, uint32_t BufOutLen, typename outT >
		class SocketClient : public eeros::Thread {
		public:
			SocketClient(std::string serverIP, uint16_t port, double period = 0.01) : read1({0}), read2({0}), read3({0}) {
				this->port = port;
				this->period = period;
				this->serverIP = serverIP;
				signal(SIGPIPE, signalHandler);	// make sure, that a broken pipe does not stop application
				read_ptr.store(&read1);
				send_ptr.store(&send1);		
				running = false;
			}
			
			virtual ~SocketClient() {
				join();
			}
			
			virtual void stop() {
				running = false;
			}
			
			virtual bool isRunning() {
				return running;
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
			
		private:
			virtual void run() {	
				log.info() << "SocketClient thread started";
				running = true;
				while (running) {
					struct sockaddr_in servAddr;
					int sockfd = socket(AF_INET, SOCK_STREAM, 0);
					if (sockfd < 0) throw Fault("ERROR opening socket");
		
					auto server = gethostbyname(serverIP.c_str()); 
					if (server == NULL) {
						throw Fault("Server ip not found");
					}
					bzero((char *) &servAddr, sizeof(servAddr));
					servAddr.sin_family = AF_INET;
					bcopy((char *)server->h_addr,(char *)&servAddr.sin_addr.s_addr, server->h_length);
					servAddr.sin_port = htons(port);
				
					while (connect(sockfd, (struct sockaddr *) &servAddr, sizeof(servAddr)) < 0) ;
					log.info() << "Client connected to ip=" << serverIP;
					inT b_write[BufInLen]; outT b_read[BufOutLen];							
					using seconds = std::chrono::duration<double, std::chrono::seconds::period>;
					auto next_cycle = std::chrono::steady_clock::now() + seconds(period);
					bool connected = true;
				
					while (connected) {
						std::this_thread::sleep_until(next_cycle);
						
						// write
						std::array<inT, BufInLen> &sendValue = getNextSendBuffer();
						for(int i = 0; i < BufInLen; i++) b_write[i] = sendValue[i]; 
//  						log.trace() << "try to write " << b_write[0];
						int n = write(sockfd, b_write, BufInLen * sizeof(inT));
						if (n < 0) {
							log.trace() << "error = " << std::strerror(errno);
							connected = false;
						}	
						
						// read
						size_t count = BufOutLen * sizeof(outT);
						uint8_t* ptr = (uint8_t *)b_read;
						auto endTime = std::chrono::steady_clock::now() + seconds(period * 100);
						while (connected && count) {
							if (std::chrono::steady_clock::now() > endTime) {
								log.trace() << "error = socket read timed out";
								connected = false;
							}
// 							log.trace() << "try to read " << count << " Bytes";
							n = read(sockfd, ptr, count);
							if (n < 0) {
								log.trace() << "error = " << std::strerror(errno);
								connected = false;
							}
							ptr += n;
							count -= n;
						}
						std::array<outT, BufOutLen> &readValue = getNextReceiveBuffer();
						for(int i = 0; i < BufOutLen; i++) readValue[i] = b_read[i];
						flip();
						next_cycle += seconds(period);
					}
					close(sockfd);
				}
			}
			
			std::array<outT, BufOutLen>& getNextReceiveBuffer() {
				auto p = read_ptr.load();
				if (p == &read1) return read2;
				else if (p == &read2) return read3;
				else if (p == &read3) return read1;
			}
			
			std::array<inT, BufInLen>& getNextSendBuffer() {
				auto p = send_ptr.load();
				if (p == &send1) return send2;
				else if (p == &send2) return send3;
				else if (p == &send3) return send1;
			}
			
			void flip() {
				read_ptr.store(&getNextReceiveBuffer());
				send_ptr.store(&getNextSendBuffer());
			}
			
			bool running;
			std::string serverIP;
			uint16_t port;
			double period;
			struct hostent *server;
			int sockfd;
			
			std::array<outT, BufOutLen> read1, read2, read3;
			std::array<inT, BufInLen> send1, send2, send3;
			std::atomic< std::array<outT, BufOutLen>* > read_ptr;
			std::atomic< std::array<inT, BufInLen>* > send_ptr;
		};

		// specialization used when server doesn't receive data from its client
		template < uint32_t BufInLen, typename inT >
		class SocketClient<BufInLen, inT, 0, std::nullptr_t> : public eeros::Thread {
		public:	
			SocketClient(std::string serverIP, uint16_t port, double period = 0.01) {
				this->port = port;
				this->period = period;
				this->serverIP = serverIP;
				signal(SIGPIPE, signalHandler);	// make sure, that a broken pipe does not stop application
				send_ptr.store(&send1);
				running = false;
			}
			
			virtual ~SocketClient() {
				join();
			}
			
			virtual void stop() {
				running = false;
			}
			
			virtual bool isRunning() {
				return running;
			}
			
			virtual void setSendBuffer(std::array<inT, BufInLen>& data) {
				auto p = send_ptr.load();
				if (p == &send1) send1 = data;
				else if (p == &send2) send2 = data;
				else if (p == &send3) send3 = data;
			}

		private:
			virtual void run() {	
				log.info() << "SocketClient thread started";
				running = true;
				while (running) {
					struct sockaddr_in servAddr;
					int sockfd = socket(AF_INET, SOCK_STREAM, 0);
					if (sockfd < 0) throw Fault("ERROR opening socket");
		
					auto server = gethostbyname(serverIP.c_str()); 
					if (server == NULL) {
						throw Fault("Server ip not found");
					}
					bzero((char *) &servAddr, sizeof(servAddr));
					servAddr.sin_family = AF_INET;
					bcopy((char *)server->h_addr,(char *)&servAddr.sin_addr.s_addr, server->h_length);
					servAddr.sin_port = htons(port);
				
					while (connect(sockfd, (struct sockaddr *) &servAddr, sizeof(servAddr)) < 0) ;
					log.info() << "Client connected to ip=" << serverIP;
					inT b_write[BufInLen]; 	
					using seconds = std::chrono::duration<double, std::chrono::seconds::period>;
					auto next_cycle = std::chrono::steady_clock::now() + seconds(period);
					bool connected = true;
				
					while (connected) {
						std::this_thread::sleep_until(next_cycle);
					
						// write
						std::array<inT, BufInLen> &sendValue = getNextSendBuffer();
						for(int i = 0; i < BufInLen; i++) b_write[i] = sendValue[i]; 
//  						log.trace() << "try to write " << b_write[0];
						int n = write(sockfd, b_write, BufInLen * sizeof(inT));
						if (n < 0) {
							log.trace() << "error = " << std::strerror(errno);
							connected = false;
						}	
				
						flip();
						next_cycle += seconds(period);
					}
					close(sockfd);
				}
			}
			
			std::array<inT, BufInLen>& getNextSendBuffer() {
				auto p = send_ptr.load();
				if (p == &send1) return send2;
				else if (p == &send2) return send3;
				else if (p == &send3) return send1;
			}
			
			void flip(){
				send_ptr.store(&getNextSendBuffer());
			}
			
			bool running;
			std::string serverIP;
			uint16_t port;
			double period;
			struct hostent *server;
			int sockfd;
			
			std::array<inT, BufInLen> send1, send2, send3;
			std::atomic< std::array<inT, BufInLen>* > send_ptr;
		};

	};
};

#endif // ORG_EEROS_SOCKET_CLIENT_HPP_