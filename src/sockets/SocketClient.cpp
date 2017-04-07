#include "Client.hpp"
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>      /* inet_ntoa() to format IP address */
#include <iostream>
#include <array>

#include <eeros/core/EEROSException.hpp>

using namespace eeros;

Client::Client(char *hostname, char *server_address) : client(hostname), server_ip(server_address), 
                                                       read_ptr(&read1), send_ptr(&send1) {
	running = false;
}

Client::~Client() {
	join();
	read1.fill(0.0);
	send1.fill(0.0);
}

void Client::stop() {
	running = false;
}

bool Client::isRunning() {
	return running;
}

const Array& Client::getBuffer() {
	return *read_ptr.load(); 
}

void Client::sendBuffer(Array data) {
	auto p = send_ptr.load();
	if (p == &send1) send1 = data;
	else if (p == &send2) send2 = data;
	else if (p == &send3) send3 = data;
}

Array& Client::getWriteBuffer() {
	auto p = read_ptr.load();
	if (p == &read1) return read2;
	else if (p == &read2) return read3;
	else if (p == &read3) return read1;
}

Array& Client::getSendBuffer() {
	auto p = send_ptr.load();
	if (p == &send1) return send2;
	else if (p == &send2) return send3;
	else if (p == &send3) return send1;
}

void Client::flip() {
	read_ptr.store(&getWriteBuffer());
	send_ptr.store(&getSendBuffer());
}

void Client::run() { 
	// Set up socket connection 
	struct sockaddr_in serv_addr;
	
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) throw EEROSException("ERROR opening socket");
	
	in_addr_t data;
	data = inet_addr(client);
	server = gethostbyname(server_ip); 
	
	if (server == NULL) {
		fprintf(stderr,"ERROR, no such host\n");
		exit(0);
	}
	bzero((char *) &serv_addr, sizeof(serv_addr));
	
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr,(char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(9876);
	
	if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
		throw EEROSException("ERROR connecting");
	
	// Start communication
	std::cout << "Client thread started" << std::endl;
	
	double b_write[4]; double b_read[4];
	int n;
	
	running = true;

	while(running){
		// 1. WRITE
		auto *sendValue = &getSendBuffer();
		for(int i=0;i<nofAxis;i++){
			b_write[i] = (*sendValue)[i];
		}
		n = write(sockfd,b_write,sizeof(b_write)*8);
		if (n < 0) throw EEROSException("ERROR writing to socket");
		
		// 2. read
		n = read(sockfd,b_read,sizeof(b_read)*8);
		if (n < 0) throw EEROSException("ERROR reading from socket");
		
		auto *readValue = &getWriteBuffer();
		for(int i=0;i<nofAxis;i++){
			(*readValue)[i] = b_read[i];
		}
		
		flip();
		readValue = &getWriteBuffer();
		sendValue = &getSendBuffer();
		
		usleep(100000);
	}
	
	close(sockfd);
}
