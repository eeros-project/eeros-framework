#include "Server.hpp"
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>      /* inet_ntoa() to format IP address */
#include <iostream>
#include <array>

#include <eeros/core/EEROSException.hpp>

using namespace eeros;

Server::Server() : read_ptr(&read1), send_ptr(&send1) {
	running = false;
}

Server::~Server() {
	join();
	read1.fill(0.0);
	send1.fill(0.0);
}

void Server::stop() {
	running = false;
}

bool Server::isRunning() {
	return running;
}

const Array& Server::getBuffer() {
	return *read_ptr.load();
}

void Server::sendBuffer(Array data) {
	auto p = send_ptr.load();
	if (p == &send1) send1 = data;
	else if (p == &send2) send2 = data;
	else if (p == &send3) send3 = data;
}

Array& Server::getWriteBuffer() {
	auto p = read_ptr.load();
	if (p == &read1) return read2;
	else if (p == &read2) return read3;
	else if (p == &read3) return read1;
}

Array& Server::getSendBuffer() {
	auto p = send_ptr.load();
	if (p == &send1) return send2;
	else if (p == &send2) return send3;
	else if (p == &send3) return send1;
}

void Server::flip() {
	read_ptr.store(&getWriteBuffer());
	send_ptr.store(&getSendBuffer());
}

void Server::run() { 
	// Set up socket connection
	int i = 0; 
	struct sockaddr_in serv_addr;
	
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) throw EEROSException("ERROR opening socket");
	
	bzero((char *) &serv_addr, sizeof(serv_addr));
	
	serv_addr.sin_port = htons(9876); 
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY) ;
	
	if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
		throw EEROSException("ERROR on binding");
	printf("\n- Connected to client IP --> %s! \n", inet_ntoa(serv_addr.sin_addr) );
	
	std::cout << "Server thread started" << std::endl;
	
	socklen_t clilen;
	listen(sockfd,1);
	struct sockaddr_in cli_addr;
	clilen = sizeof(cli_addr);
	newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr,  &clilen);
	if (newsockfd < 0) throw EEROSException("ERROR on accept");
	
	double b_write[4]; double b_read[4];
	int n;
	
	running = true;
	
	while(running){
		// 1. READ
		n = read(newsockfd,b_read,sizeof(b_read)*8);
		if (n < 0) throw EEROSException("ERROR reading from socket");
 		
		auto *readValue = &getWriteBuffer();
		for(int i=0;i<nofAxis;i++){
			(*readValue)[i] = b_read[i];
		}
 		
 		// 2. SEND
 		auto *sendValue = &getSendBuffer();
		for(int i=0;i<nofAxis;i++){
			b_write[i] = (*sendValue)[i]; 
		}
		n = write(newsockfd,b_write,sizeof(b_write)*8);
		if (n < 0) throw EEROSException("ERROR writing to socket");

		flip();
		readValue = &getWriteBuffer();
		sendValue = &getSendBuffer();
		
		usleep(100000);
	}
		
	close(newsockfd);
	close(sockfd);
}
