#include <eeros/hal/ComediDac.hpp>

using namespace eeros::hal;

ComediDac::ComediDac(std::string id, ComediDevice* device, uint32_t subDeviceNumber, uint32_t channel, double scale, double offset) : SystemOutput<double>(id) {
	this->deviceHandle = device->getDeviceHandle();
	this->subDeviceNumber = subDeviceNumber;
	this->channel = channel;
	this->scale = scale;
	this->offset = offset;
}

double ComediDac::get() {
	lsampl_t data = 0;
	comedi_data_read(deviceHandle, subDeviceNumber, channel, 0, AREF_GROUND, &data);
	return (static_cast<int>(data) - offset) / scale;
}

void ComediDac::set(double value) {
	lsampl_t data = static_cast<lsampl_t>(value * maxVal / 20.0 + maxVal / 2.0);
	comedi_data_write(deviceHandle, subDeviceNumber, channel, 0, AREF_GROUND, data);
}
