#include <eeros/hal/ComediDigOut.hpp>

using namespace eeros::hal;

ComediDigOut::ComediDigOut(std::string id, ComediDevice* device, uint32_t subDeviceNumber, uint32_t channel, bool inverted) : PeripheralOutput<bool>(id) {
	this->deviceHandle = device->getDeviceHandle();
	this->subDeviceNumber = subDeviceNumber;
	this->channel = channel;
	this->inverted = inverted;
	
	comedi_dio_config(deviceHandle, subDeviceNumber, channel, COMEDI_OUTPUT);
}

bool ComediDigOut::get() const {
	lsampl_t data = 0;
	bool value;
	comedi_dio_read(deviceHandle, subDeviceNumber, channel, &data);
	value = static_cast<bool>(data);
	if(inverted) value = !value;
	return value;
}

void ComediDigOut::set(bool value) {
	if(inverted) value = !value;
	comedi_dio_write(deviceHandle, subDeviceNumber, channel, value);
}
