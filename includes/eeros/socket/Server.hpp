#ifndef CH_NTB_SOCKET_SERVER_HPP_
#define CH_NTB_SOCKET_SERVER_HPP_

#include <eeros/core/Thread.hpp>
#include <atomic>
#include <array>
#include <unistd.h>
#include <netdb.h> 

#include "../control/constants.hpp"
#include "../control/types.hpp"

using namespace eeduro::delta;

class Server : public eeros::Thread {

public:
	
	Server();
	virtual ~Server();
	
	virtual void stop();
	virtual bool isRunning();
	virtual const Array& getBuffer();
	virtual void sendBuffer(Array data);

private:
	virtual void run();
	Array& getWriteBuffer();
	Array& getSendBuffer();
	void flip();
	
	bool running;
	struct hostent *server;
	int sockfd, newsockfd; 
	
	Array read1; Array read2; Array read3;
	Array send1; Array send2; Array send3;

	std::atomic<Array*> read_ptr;
	std::atomic<Array*> send_ptr;
};

#endif // CH_NTB_SOCKET_SERVER_HPP_