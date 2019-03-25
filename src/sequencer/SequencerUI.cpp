#include <eeros/sequencer/SequencerUI.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/core/Fault.hpp>
#include <unistd.h>
#include <netdb.h> 
#include <arpa/inet.h> 		/* inet_ntoa() to format IP address */
#include <string.h>
#include <sys/socket.h>
#include <iostream>
#include <cstring>
#include <signal.h>


namespace eeros {
	namespace sequencer {

		void sigPipeHandler(int signum) {std::cout << "SIGPIPE received" << std::endl; }
		
		SequencerUI::SequencerUI() {
			this->port = 7799;
			this->period = 0.01;
			signal(SIGPIPE, sigPipeHandler);	// make sure, that a broken pipe does not stop application
			running = false;
		}
			
		SequencerUI::~SequencerUI() {
			join();
		}
			
		void SequencerUI::stop() {
			running = false;
		}
			
		bool SequencerUI::isRunning() {
			return running;
		}
			
		void SequencerUI::run() {	
			log.info() << "SequencerUI thread started";
			struct sockaddr_in servAddr;
			sockfd = socket(AF_INET, SOCK_STREAM, 0);
			if (sockfd < 0) throw Fault("ERROR opening socket");
				
			bzero((char *) &servAddr, sizeof(servAddr));	
			servAddr.sin_port = htons(port); 
			servAddr.sin_family = AF_INET;
			servAddr.sin_addr.s_addr = htonl(INADDR_ANY) ;
			if (bind(sockfd, (struct sockaddr *) &servAddr, sizeof(servAddr)) < 0) 
				throw Fault("ERROR on binding");
			
			socklen_t clilen;
			listen(sockfd,1);
			struct sockaddr_in cliAddr;
			clilen = sizeof(cliAddr);
			
			running = true;
			while (running) {
				fd_set fds;
				int res;
				FD_ZERO (&fds);
				FD_SET (sockfd, &fds);
				struct timeval timeout;
				timeout.tv_sec = 0;
				timeout.tv_usec = 50000;
				if (select(sockfd + 1, &fds, NULL, NULL, &timeout) > 0) {
					newsockfd = accept(sockfd, (struct sockaddr *) &cliAddr,  &clilen);
					if (newsockfd < 0) throw Fault("ERROR on accept");
					bool connected = true;
					char cliName[INET6_ADDRSTRLEN];
					getnameinfo((struct sockaddr*)&cliAddr, sizeof cliAddr, cliName, sizeof(cliName), NULL, 0, NI_NUMERICHOST|NI_NUMERICSERV);
					log.info() << "Client connection from ip=" << cliName << " accepted";
					char ch;
					using seconds = std::chrono::duration<double, std::chrono::seconds::period>;
					auto next_cycle = std::chrono::steady_clock::now() + seconds(period);
				
					while (connected) {
						std::this_thread::sleep_until(next_cycle);
									
						size_t count = 1;
						auto endTime = std::chrono::steady_clock::now() + seconds(period * 100);
						while (connected && count) {
							if (std::chrono::steady_clock::now() > endTime) {
								log.trace() << "error = socket read timed out";
								connected = false;
							}
	// 						log.trace() << "try to read " << count << " Bytes";
							int n = read(newsockfd, &ch, count);
							if (n < 0) {
								log.trace() << "error = " << std::strerror(errno);
								connected = false;
							}
							count -= n;
						}
						Sequencer& seq = Sequencer::instance();
						switch (ch) {
							case 's': 
								seq.singleStepping();
								seq.step();
								break;
							case 'c':
								seq.restart();
								break;
							case 'b':
								seq.singleStepping();
								break;
							default: ;
						}
						next_cycle += seconds(period);
					}
					close(newsockfd);
				}
			}
			close(sockfd);
		}

	}
}
