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

namespace eeros {
	namespace sockets {
				
		template < uint32_t BufInLen, typename inT, uint32_t BufOutLen, typename outT >
		class SocketServer : public eeros::Thread {
		public:
			
			SocketServer(uint16_t port, double period = 0.01) : read1({0}), read2({0}), read3({0}) {
				this->port = port;
				this->period = period;
				
				read_ptr.store(&read1);
				send_ptr.store(&send1);
				
				running = false;
			}
			
			virtual ~SocketServer(){
				join();
			}
			
			virtual void stop(){
				running = false;
			}
			
			virtual bool isRunning(){
				return running;
			}
			
			virtual std::array<outT, BufOutLen>& getReceiveBuffer(){
				return *read_ptr.load();
			}
			
			virtual void setSendBuffer(std::array<inT, BufInLen>& data){
				auto p = send_ptr.load();
				if (p == &send1) send1 = data;
				else if (p == &send2) send2 = data;
				else if (p == &send3) send3 = data;
			}

			outT readbuffer;
			
		private:
			virtual void run(){	
				// Set up socket connection
				int i = 0; 
				struct sockaddr_in serv_addr;
				
				sockfd = socket(AF_INET, SOCK_STREAM, 0);
				if (sockfd < 0) throw Fault("ERROR opening socket");
				
				bzero((char *) &serv_addr, sizeof(serv_addr));
				
				serv_addr.sin_port = htons(port); 
				serv_addr.sin_family = AF_INET;
				serv_addr.sin_addr.s_addr = htonl(INADDR_ANY) ;
				
				// bind()
				if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
					throw Fault("ERROR on binding");
				log.info() << "- Connected to client IP --> " << inet_ntoa(serv_addr.sin_addr);
				log.info() << "SocketServer thread started";
				
				// listen()
				socklen_t clilen;
				listen(sockfd,1);
				struct sockaddr_in cli_addr;
				clilen = sizeof(cli_addr);
				// accept()
				newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr,  &clilen);
				if (newsockfd < 0) throw Fault("ERROR on accept");
				
				log.info() << "SocketServer connection accepted";
				inT b_write[BufInLen]; outT b_read[BufOutLen];
				
				running = true;
				
				using seconds = std::chrono::duration<double, std::chrono::seconds::period>;
				auto next_cycle = std::chrono::steady_clock::now() + seconds(period);
				
				while(running) {
					int n;
					std::this_thread::sleep_until(next_cycle);
					
					// write
					std::array<inT, BufInLen> &sendValue = getNextSendBuffer();
					for(int i = 0; i < BufInLen; i++) b_write[i] = sendValue[i]; 
					n = write(newsockfd, b_write, BufInLen * sizeof(inT));
					if (n < 0) throw Fault("ERROR writing to socket");
					
					// read
					size_t count = BufOutLen * sizeof(outT);
					uint8_t* ptr = (uint8_t *)b_read;
					while (count) {
// 						log.trace() << "try to read " << count << " Bytes";
						n = read(newsockfd, ptr, count);
// 						log.trace() << "could read " << n << " Bytes";
						if (n < 0) {
							log.trace() << "error = " << std::strerror(errno);
							throw Fault("ERROR reading from socket");
						}
						ptr += n;
						count -= n;
					}
					std::array<outT, BufOutLen> &readValue = getNextReceiveBuffer();
					for(int i = 0; i < BufOutLen; i++) readValue[i] = b_read[i];
			
					flip();
					
					next_cycle += seconds(period);
				}
				close(newsockfd);
				close(sockfd);
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
			
			void flip(){
				read_ptr.store(&getNextReceiveBuffer());
				send_ptr.store(&getNextSendBuffer());
			}
			
			bool running;
			uint16_t port;
			double period;
			struct hostent *server;
			int sockfd;
			int newsockfd;
			
			std::array<outT, BufOutLen> read1, read2, read3;
			std::array<inT, BufInLen> send1, send2, send3;
			std::atomic< std::array<outT, BufOutLen>* > read_ptr;
			std::atomic< std::array<inT, BufInLen>* > send_ptr;
		};

		template < uint32_t BufInLen, typename inT >
		class SocketServer<BufInLen, inT, 0, std::nullptr_t> : public eeros::Thread {
		public:
			
			SocketServer(uint16_t port, double period = 0.01) {
				this->port = port;
				this->period = period;
				
				send_ptr.store(&send1);
				
				running = false;
			}
			
			virtual ~SocketServer(){
				join();
			}
			
			virtual void stop(){
				running = false;
			}
			
			virtual bool isRunning(){
				return running;
			}
			
			virtual void setSendBuffer(std::array<inT, BufInLen>& data){
				auto p = send_ptr.load();
				if (p == &send1) send1 = data;
				else if (p == &send2) send2 = data;
				else if (p == &send3) send3 = data;
			}

		private:
			virtual void run(){	
				// Set up socket connection
				int i = 0; 
				struct sockaddr_in serv_addr;
				
				sockfd = socket(AF_INET, SOCK_STREAM, 0);
				if (sockfd < 0) throw Fault("ERROR opening socket");
				
				bzero((char *) &serv_addr, sizeof(serv_addr));
				
				serv_addr.sin_port = htons(port); 
				serv_addr.sin_family = AF_INET;
				serv_addr.sin_addr.s_addr = htonl(INADDR_ANY) ;
				
				// bind()
				if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
					throw Fault("ERROR on binding");
				log.info() << "- Connected to client IP --> " << inet_ntoa(serv_addr.sin_addr);
				log.info() << "SocketServer thread started";
				
				// listen()
				socklen_t clilen;
				listen(sockfd,1);
				struct sockaddr_in cli_addr;
				clilen = sizeof(cli_addr);
				// accept()
				newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr,  &clilen);
				if (newsockfd < 0) throw Fault("ERROR on accept");
				
				log.info() << "SocketServer connection accepted";
				inT b_write[BufInLen]; 
				
				running = true;
				
				using seconds = std::chrono::duration<double, std::chrono::seconds::period>;
				auto next_cycle = std::chrono::steady_clock::now() + seconds(period);
				
				while(running) {
					int n;
					std::this_thread::sleep_until(next_cycle);
					
					// write
					std::array<inT, BufInLen> &sendValue = getNextSendBuffer();
					for(int i = 0; i < BufInLen; i++) b_write[i] = sendValue[i]; 
					n = write(newsockfd, b_write, BufInLen * sizeof(inT));
					if (n < 0) throw Fault("ERROR writing to socket");
// 						log.trace() << "write" << " bbbbbbbbbbbbbbbbbbbbb";
				
					flip();
					
					next_cycle += seconds(period);
				}
				close(newsockfd);
				close(sockfd);
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
			uint16_t port;
			double period;
			struct hostent *server;
			int sockfd;
			int newsockfd;
			
			std::array<inT, BufInLen> send1, send2, send3;
			std::atomic< std::array<inT, BufInLen>* > send_ptr;
		};

	};
};

#endif // ORG_EEROS_SOCKET_SERVER_HPP_