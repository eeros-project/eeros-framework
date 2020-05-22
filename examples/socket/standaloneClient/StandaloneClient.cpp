#include <iostream>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>      /* inet_ntoa() to format IP address */
#include <iostream>
#include <array>
#include <unistd.h>
#include <netdb.h>
#include <chrono>
#include <thread>
#include <arpa/inet.h>


int main(int argc, char **argv) {
	char* serverIP = (char*)"127.0.0.1";
	int port = 9876;
	double period = 0.1;
	
	while (1) {
		std::cout << "Client thread started" << std::endl;
		struct sockaddr_in servAddr;
		int sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0) std::cout << "ERROR opening socket" << std::endl;
		
		auto server = gethostbyname(serverIP); 
		if (server == NULL) {
			std::cout << "ERROR, no such host\n" << std::endl;
			exit(0);
		}
		bzero((char *) &servAddr, sizeof(servAddr));
		servAddr.sin_family = AF_INET;
		bcopy((char *)server->h_addr,(char *)&servAddr.sin_addr.s_addr, server->h_length);
		servAddr.sin_port = htons(port);
		
		while (connect(sockfd, (struct sockaddr *) &servAddr, sizeof(servAddr)) < 0) ;
		std::cout << "Client connected to ip=" << serverIP << std::endl;
		using seconds = std::chrono::duration<double, std::chrono::seconds::period>;
		auto next_cycle = std::chrono::steady_clock::now() + seconds(period);
		bool connected = true;
		
		double readBuf[4];
		double writeBuf[6]; 
		int n;
		
		double dataToSend = 0.1;

		while (connected) {
			std::this_thread::sleep_until(next_cycle);
			
			// write
			std::cout << "w: ";
			for (uint32_t i = 0; i < sizeof(writeBuf)/sizeof(writeBuf[0]); i++) {
				writeBuf[i] = dataToSend;
				dataToSend += 0.1;
				std::cout << writeBuf[i] << "\t";
			}
			std::cout << std::endl;
			n = write(sockfd, writeBuf, sizeof(writeBuf));
			if (n < 0) {
				std::cout << "ERROR writing to socket" << std::endl;
				connected = false;
			}
			
			// read
			n = read(sockfd, readBuf, sizeof(readBuf));
			if (n < 0) std::cout << "ERROR reading from socket" << std::endl;
			
			std::cout << "rec: ";
			for (uint32_t i = 0; i < sizeof(readBuf)/sizeof(readBuf[0]); i++) {
				std::cout << readBuf[i] << "\t";
			}
			std::cout << std::endl;
			next_cycle += seconds(period);
		}
		close(sockfd);
	}
	
    return 0;
}
