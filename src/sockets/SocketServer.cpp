#include <eeros/sockets/SocketServer.hpp>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>      /* inet_ntoa() to format IP address */
#include <iostream>
#include <array>
#include <sys/socket.h>
#include <fcntl.h>
// #include <eeros/core/EEROSException.hpp>

#include <string.h>
#include <netdb.h>
#include <arpa/inet.h>
// #include "tcpconnector.h"
 


using namespace eeros;
using namespace eeros::sockets;

SocketServer::SocketServer(uint16_t port, int byteSize, double periodInSec) :
port(port), byteSize(byteSize), periodInSec(periodInSec)
// ,read_ptr(&read1), send_ptr(&send1)
{
	running = false;
}

SocketServer::~SocketServer() {
	join();
// 	read1.fill(0.0);
// 	send1.fill(0.0);
}

void SocketServer::stop() {
	running = false;
}

bool SocketServer::isRunning() {
	return running;
}

// const Array& SocketServer::getBuffer() {
// 	return *read_ptr.load();
// }
// 
// void SocketServer::sendBuffer(Array data) {
// 	auto p = send_ptr.load();
// 	if (p == &send1) send1 = data;
// 	else if (p == &send2) send2 = data;
// 	else if (p == &send3) send3 = data;
// }
// 
// Array& SocketServer::getWriteBuffer() {
// 	auto p = read_ptr.load();
// 	if (p == &read1) return read2;
// 	else if (p == &read2) return read3;
// 	else if (p == &read3) return read1;
// }
// 
// Array& SocketServer::getSendBuffer() {
// 	auto p = send_ptr.load();
// 	if (p == &send1) return send2;
// 	else if (p == &send2) return send3;
// 	else if (p == &send3) return send1;
// }
// 
// void SocketServer::flip() {
// 	read_ptr.store(&getWriteBuffer());
// 	send_ptr.store(&getSendBuffer());
// }

void SocketServer::run() { 
	// Set up socket connection
	int i = 0; 
	struct sockaddr_in serv_addr;
	
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
// 	if (sockfd < 0) throw EEROSException("ERROR opening socket");
	if (sockfd < 0) log.error() << "ERROR opening socket";
	
	bzero((char *) &serv_addr, sizeof(serv_addr));
	
	serv_addr.sin_port = htons(port); 
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY) ;
	
	// bind()
	if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
// 		throw EEROSException("ERROR on binding");
		log.error() << "ERROR on binding";
// 	printf("\n- Connected to client IP --> %s! \n", inet_ntoa(serv_addr.sin_addr) );
	log.info() << "- Connected to client IP --> " << inet_ntoa(serv_addr.sin_addr);
	
// 	std::cout << "SocketServer thread started" << std::endl;
	log.info() << "SocketServer thread started";
	
	// listen()
	socklen_t clilen;
	listen(sockfd,1);
	struct sockaddr_in cli_addr;
	clilen = sizeof(cli_addr);
	// accept()
	newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr,  &clilen);
// 	if (newsockfd < 0) throw EEROSException("ERROR on accept");
	if (newsockfd < 0) log.error() << "ERROR on accept";
	
	double b_write[4]; double b_read[4];
	int n;
	
	running = true;
	
	using seconds = std::chrono::duration<double, std::chrono::seconds::period>;
	auto next_cycle = std::chrono::steady_clock::now() + seconds(periodInSec);
	
	while(running){
		std::this_thread::sleep_until(next_cycle);
		
// 		// read()
// 		n = read(newsockfd,b_read,sizeof(b_read)*8);
// // 		if (n < 0) throw EEROSException("ERROR reading from socket");
// 		if (n < 0) log.error() << "ERROR reading from socket";
//  		
// 		auto *readValue = &getWriteBuffer();
// 		for(int i=0;i<nofAxis;i++){
// 			(*readValue)[i] = b_read[i];
// 		}
		
		// read()
		n = read(newsockfd, &readbuffer, 8);
// 		if (n < 0) throw EEROSException("ERROR reading from socket");
		if (n < 0) log.error() << "ERROR reading from socket";
 		
// 		auto *readValue = &getWriteBuffer();
// 		for(int i=0;i<nofAxis;i++){
// 			(*readValue)[i] = b_read[i];
// 		}
 		
//  		// write()
//  		auto *sendValue = &getSendBuffer();
// 		for(int i=0;i<nofAxis;i++){
// 			b_write[i] = (*sendValue)[i]; 
// 		}
// 		n = write(newsockfd,b_write,sizeof(b_write)*8);
// // 		if (n < 0) throw EEROSException("ERROR writing to socket");
// 		if (n < 0) log.error() << "ERROR writing to socket";

// 		flip();
// 		readValue = &getWriteBuffer();
// 		sendValue = &getSendBuffer();
		
// 		usleep(100000);
		next_cycle += seconds(periodInSec);
	}
		
	close(newsockfd);
	close(sockfd);
}
