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

namespace eeros {
	namespace sockets {
		
		template < uint8_t BufLen = 32, typename T = double >
		class SocketServer : public eeros::Thread {

		public:
			
			SocketServer(uint16_t port, double period = 0.01) : read1({0.0}), read2({0.0}), read3({0.0}) {
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
			
			virtual std::array<T, BufLen>& getReceiveBuffer(){
				return *read_ptr.load();
			}
			virtual void setSendBuffer(std::array<T, BufLen>& data){
				auto p = send_ptr.load();
				if (p == &send1) send1 = data;
				else if (p == &send2) send2 = data;
				else if (p == &send3) send3 = data;
			}

			T readbuffer;
			
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
				T b_write[BufLen]; T b_read[BufLen];
				int n;
				
				running = true;
				
				using seconds = std::chrono::duration<double, std::chrono::seconds::period>;
				auto next_cycle = std::chrono::steady_clock::now() + seconds(period);
				
				while(running){
					std::this_thread::sleep_until(next_cycle);
					
					// read
					n = read(newsockfd,b_read,BufLen * sizeof(T));
					if (n < 0) throw Fault("ERROR reading from socket");
					
					std::array<T, BufLen> &readValue = getNextReceiveBuffer();
					for(int i=0;i < BufLen ;i++){
						readValue[i] = b_read[i];
					}
					
					// write
					std::array<T, BufLen> &sendValue = getNextSendBuffer();
					for(int i=0;i < BufLen;i++){
						b_write[i] = sendValue[i]; 
					}
					n = write(newsockfd,b_write,BufLen * sizeof(T));
					if (n < 0) throw Fault("ERROR writing to socket");
			
					flip();
					
					next_cycle += seconds(period);
				}
					
				close(newsockfd);
				close(sockfd);
			}
			
			std::array<T, BufLen>& getNextReceiveBuffer() {
				auto p = read_ptr.load();
				if (p == &read1) return read2;
				else if (p == &read2) return read3;
				else if (p == &read3) return read1;
			}
			
			std::array<T, BufLen>& getNextSendBuffer() {
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
			
			
			std::array<T, BufLen> read1, read2, read3;
			std::array<T, BufLen> send1, send2, send3;

			std::atomic< std::array<T, BufLen>* > read_ptr;
			std::atomic< std::array<T, BufLen>* > send_ptr;
			
		};
	};
};

#endif // ORG_EEROS_SOCKET_SERVER_HPP_