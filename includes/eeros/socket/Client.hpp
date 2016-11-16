#ifndef CH_NTB_SOCKET_CLIENT_HPP_
#define CH_NTB_SOCKET_CLIENT_HPP_

#include <eeros/core/Thread.hpp>
#include <atomic>
#include <array>
#include <unistd.h>
#include <netdb.h> 

#include "../control/constants.hpp"
#include "../control/types.hpp"

using namespace eeduro::delta;

class Client : public eeros::Thread {

public:
	Client(char *hostname, char *server_address);
	virtual ~Client();
	
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
	char *client, *server_ip;
    struct hostent *server;
	int sockfd; 
	
	Array read1; Array read2; Array read3;
	Array send1; Array send2; Array send3;

	std::atomic<Array*> read_ptr;
	std::atomic<Array*> send_ptr;
};

#endif // CH_NTB_SOCKET_CLIENT_HPP_