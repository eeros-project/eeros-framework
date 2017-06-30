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
	char* server_ip;
	int port;
	
	
	server_ip = "0.0.0.0";
	server_ip = "127.0.0.1";
	port = 9876;
	
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
	
	double b_write[4]; double b_read[4];
	int n;
	
	double dataToSend = 0;

	while(1){
		// 1. WRITE
		std::cout << "w: ";
		for(int i = 0; i < sizeof(b_write)/sizeof(b_write[0]); i++){
			b_write[i] = dataToSend;
			dataToSend++;
			std::cout << b_write[i] << "\t";
		}
		std::cout << std::endl;
		n = write(sockfd,b_write,sizeof(b_write)*8);
		if (n < 0) std::cout << "ERROR writing to socket" << std::endl;
		
		// 2. READ
		n = read(sockfd,b_read,sizeof(b_read)*8);
		if (n < 0) std::cout << "ERROR reading from socket" << std::endl;
		
		std::cout << "rec: ";
		for(int i=0;i<4;i++){
			std::cout << b_read[i] << "\t";
		}
		std::cout << std::endl;
		
		usleep(100000);
	}
	
	close(sockfd);
	
    return 0;
}
