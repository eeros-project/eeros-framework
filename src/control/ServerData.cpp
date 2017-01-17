#include "ServerData.hpp"
#include <eeros/core/System.hpp>
#include <iostream>

using namespace eeduro::delta;
using namespace eeros::control;
using namespace eeros;

ServerData::ServerData(Server* serverThread) : server(serverThread) { }

Input<AxisVector>& ServerData::getIn() {
	return in;
}

Output<AxisVector>& ServerData::getOut() {
	return out;
}

void ServerData::run() {
	timestamp_t time = System::getTimeNs();
	std::array<double,nofAxis> getData, sendData; 
	AxisVector output; 
	
	// Get data from other robot
	getData = server->getBuffer(); 
	output << getData[0], getData[1], getData[2], getData[3];
	
	// Send data to other robot
	for(int i=0;i<nofAxis;i++){
// 		sendData[i] = 5.0;
		sendData[i] = in.getSignal().getValue()(i);
	}
	server->sendBuffer(sendData);
	
	out.getSignal().setValue(output);
	out.getSignal().setTimestamp(time);

}
