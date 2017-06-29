#include <eeros/control/ServerData.hpp>
#include <eeros/core/System.hpp>
#include <iostream>
#include <eeros/sockets/SocketServer.hpp>

using namespace eeros::control;
using namespace eeros;

template < typename T >
ServerData<T>::ServerData(eeros::sockets::SocketServer<T>* serverThread) : server(serverThread) { }

template < typename T >
Input<T>& ServerData<T>::getIn() {
	return in;
}

template < typename T >
Output<T>& ServerData<T>::getOut() {
	return out;
}

template < typename T >
void ServerData<T>::run() {
	timestamp_t time = System::getTimeNs();
	std::array<T, sizeof(T)> getData, sendData; 
	T output; 
	
	// Get data from other robot
	getData = server->getBuffer(); 
	output << getData[0], getData[1], getData[2], getData[3];
	
	// Send data to other robot
	for(int i=0;i<sendData.size();i++){
// 		sendData[i] = 5.0;
		sendData[i] = in.getSignal().getValue()(i);
	}
	server->sendBuffer(sendData);
	
	out.getSignal().setValue(output);
	out.getSignal().setTimestamp(time);

}
