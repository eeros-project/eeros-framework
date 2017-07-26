#include <iostream>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>      /* inet_ntoa() to format IP address */
#include <iostream>
#include <array>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>


int main(int argc, char **argv) {
	char* server_ip = (char*)"127.0.0.1";
	int port = 9876;
	
	struct sockaddr_in serv_addr;
	
	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) std::cout << "ERROR opening socket" << std::endl;
	
	auto server = gethostbyname(server_ip); 
	
	if (server == NULL) {
		std::cout << "ERROR, no such host\n" << std::endl;
		exit(0);
	}
	bzero((char *) &serv_addr, sizeof(serv_addr));
	
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr,(char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(port);
	
	if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
		std::cout << "ERROR connecting" << std::endl;
	
	// Start communication
	std::cout << "Client thread started" << std::endl;
	
	double readBuf[4];
	double writeBuf[6]; 
	int n;
	
	double dataToSend = 0.1;

	while(1) {
		// 1. WRITE
		std::cout << "w: ";
		for (int i = 0; i < sizeof(writeBuf)/sizeof(writeBuf[0]); i++) {
			writeBuf[i] = dataToSend;
			dataToSend += 0.1;
			std::cout << writeBuf[i] << "\t";
		}
		std::cout << std::endl;
		n = write(sockfd, writeBuf, sizeof(writeBuf));
		if (n < 0) std::cout << "ERROR writing to socket" << std::endl;
		
		// 2. READ
		n = read(sockfd, readBuf, sizeof(readBuf));
		if (n < 0) std::cout << "ERROR reading from socket" << std::endl;
		
		std::cout << "rec: ";
		for (int i = 0; i < sizeof(readBuf)/sizeof(readBuf[0]); i++) {
			std::cout << readBuf[i] << "\t";
		}
		std::cout << std::endl;
		
		usleep(100000);
	}
	
	close(sockfd);
	
    return 0;
}
