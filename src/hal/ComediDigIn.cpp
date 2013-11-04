#include <eeros/hal/ComediDigIn.hpp>

using namespace eeros::hal;

ComediDigIn::ComediDigIn(std::string id, ComediDevice& device, uint32_t subDeviceNumber, uint32_t channel, bool inverted = false) : SystemInput<bool>(id) {
	this->deviceHandle = device.getDeviceHandle();
	this->subDeviceNumber = subDeviceNumber;
	this->channel = channel;
	this->inverted = inverted;
	
	comedi_dio_config(deviceHandle, subDeviceNumber, channel, COMEDI_INPUT);
}

double ComediDigIn::get() {
	lsampl_t data = 0;
	data = comedi_dio_read(deviceHandle, subDeviceNumber, channel, &data);
	if(inverted) data = !data;
	return static_cast<bool>(data);
}