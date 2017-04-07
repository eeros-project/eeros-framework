#include <eeros/control/ClientData.hpp>
#include <eeros/core/System.hpp>

#include <iostream>

using namespace eeduro::delta;
using namespace eeros::control;
using namespace eeros;

ClientData::ClientData(Client* clientThread) : client(clientThread) { }

Input<AxisVector>& ClientData::getIn() {
	return in;
}

Output<AxisVector>& ClientData::getOut() {
	return out;
}

void ClientData::run() {
	timestamp_t time = System::getTimeNs();
	std::array<double,nofAxis> getData, sendData; 
	AxisVector output; 

	// Get data from other robot
	getData = client->getBuffer(); 
	output << getData[0], getData[1], getData[2], getData[3];
	
	// Send data to other robot
	for(int i=0;i<nofAxis;i++){
// 		sendData[i] = 4.0;
		sendData[i] = in.getSignal().getValue()(i);
	}
	client->sendBuffer(sendData);
	
	out.getSignal().setValue(output);
	out.getSignal().setTimestamp(time);

}
