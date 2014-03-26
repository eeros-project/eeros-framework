#include <eeros/hal/FlinkPwm.hpp>

using namespace eeros::hal;

FlinkPwm::FlinkPwm(std::string id,
				   FlinkDevice* device,
				   uint32_t subDeviceNumber,
				   uint32_t channel,
				   double scale,
				   double offset) : ScalablePeripheralOutput<double>(id, scale, offset) {
	this->deviceHandle = device->getDeviceHandle();
	this->subDeviceNumber = subDeviceNumber;
	this->channel = channel;
}

void FlinkPwm::set(double frequency, double dutyCycle) {
	// TODO
	//lsampl_t data = static_cast<lsampl_t>(value * maxValue / (maxVoltage - minVoltage) + maxValue / 2.0);
	//comedi_data_write(deviceHandle, subDeviceNumber, channel, 0, AREF_GROUND, data);
}
