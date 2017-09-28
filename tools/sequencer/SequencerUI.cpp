#include <iostream>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>      /* inet_ntoa() to format IP address */
#include <array>
#include <unistd.h>
#include <netdb.h>
#include <chrono>
#include <thread>
#include <arpa/inet.h>


int main(int argc, char *argv[]) {
	char* serverIP;
	int port = 7799;
	double period = 0.1;

	// Error message if long dashes (en dash) are used
	for (int i = 0; i < argc; i++) {
		 if ((argv[i][0] == 226) && (argv[i][1] == 128) && (argv[i][2] == 147)) {
			std::cerr << "Error: Invalid arguments. En dashes are used." << std::endl;
			return -1;
		 }
	}
	
	/* Compute command line arguments */
	if (argc < 3) {
		std::cerr << "Usage: " << argv[0] << " <option(s)> " << "Options:\n"
			<< "\t-h,--help\t\tShow this help message\n"
			<< "\t-i,--serverIP SERVER IP\tSpecify the server IP address"
			<< std::endl;
		return 1;
	}
	for (int i = 1; i < argc; i++) {
		std::string arg = argv[i];
		if ((arg == "-h") || (arg == "--help")) {
			std::cerr << "Usage: " << argv[0] << " <option(s)> " << "Options:\n"
			<< "\t-h,--help\t\tShow this help message\n"
			<< "\t-i,--serverIP SERVER IP\tSpecify the server IP address"
			<< std::endl;
			return 0;
		} else if ((arg == "-i") || (arg == "--serverIP")) {
			if (i + 1 < argc) { // Make sure we aren't at the end of argv!
				i++;
				serverIP = argv[i]; // Increment 'i' so we don't get the argument as the next argv[i].
			} else { // Uh-oh, there was no argument to the destination option.
				std::cerr << "--destination option requires one argument." << std::endl;
				return 1;
			}  
			} else {
		}
	}
	std::cout << "Sequencer UI started, try to connect to ip=" << serverIP << std::endl;
	while (1) {
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
		
		uint8_t ch; 
		int n;
		
		while (connected) {
			std::this_thread::sleep_until(next_cycle);
			std::cout << "Enter 's' for single step, 'c' for continue, 'b' for break: ";
			std::string input = "";
			std::getline(std::cin, input);
			if (input.length() == 1) {
				switch (input[0]) {
					case 's': 
					case 'c':
					case 'b':
						ch = input[0];
						break;
					default:
						ch = 0;
						std::cout << "Invalid character" << std::endl;
				}
				if (ch != 0) {
					n = write(sockfd, &ch, sizeof(ch));
					if (n < 0) {
						std::cout << "ERROR writing to socket" << std::endl;
						connected = false;
					}
				}
			} else std::cout << "Invalid character" << std::endl;
			
			next_cycle += seconds(period);
		}
		close(sockfd);
	}
	
    return 0;
}
