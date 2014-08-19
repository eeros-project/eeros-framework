#include <eeros/hal/ComediDac.hpp>

using namespace eeros::hal;

ComediDac::ComediDac(std::string id,
					 ComediDevice* device,
					 uint32_t subDeviceNumber,
					 uint32_t channel,
					 double scale,
					 double offset) : ScalablePeripheralOutput<double>(id, scale, offset) {
	this->deviceHandle = device->getDeviceHandle();
	this->subDeviceNumber = subDeviceNumber;
	this->channel = channel;
	this->minVoltage = -10.0; // default +/-10V
	this->maxVoltage = 10.0;
	this->maxValue = 65535; // default 16bit
}

double ComediDac::get() const {
	lsampl_t data = 0;
	comedi_data_read(deviceHandle, subDeviceNumber, channel, 0, AREF_GROUND, &data);
	return (static_cast<int>(data) - offset) / scale;
}

void ComediDac::set(double value) {
	if(value > maxVoltage) value = maxVoltage;
	if(value < minVoltage) value = minVoltage;
	lsampl_t data = static_cast<lsampl_t>(value * maxValue / (maxVoltage - minVoltage) + maxValue / 2.0);
	comedi_data_write(deviceHandle, subDeviceNumber, channel, 0, AREF_GROUND, data);
}

void ComediDac::setVoltageRange(double minVoltage, double maxVoltage) {
	this->minVoltage = minVoltage;
	this->maxVoltage = maxVoltage;
}


void ComediDac::setDacResolution(uint8_t bits) {
	this->maxValue = (1 << bits) - 1;
}
