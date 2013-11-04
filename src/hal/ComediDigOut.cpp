#include <eeros/hal/ComediDigOut.hpp>

using namespace eeros::hal;

ComediDigOut::ComediDigOut(std::string id, ComediDevice& device, uint32_t subDeviceNumber, uint32_t channel, bool inverted) : SystemOutput<bool>(id) {
	this->deviceHandle = device.getDeviceHandle();
	this->subDeviceNumber = subDeviceNumber;
	this->channel = channel;
	this->inverted = inverted;
	
	comedi_dio_config(deviceHandle, subDeviceNumber, channel, COMEDI_OUTPUT);
}

double ComediDigOut::get() {
	lsampl_t data = 0;
	data = comedi_dio_read(deviceHandle, subDeviceNumber, channel, &data);
	if(inverted) data = !data;
	return static_cast<bool>(data);
}

void ComediDigOut::set(bool value) {
	comedi_dio_write(deviceHandle, subDeviceNumber, channel, value);
}
