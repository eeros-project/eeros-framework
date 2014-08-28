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
	flink_pwm_get_baseclock(deviceHandle, subDeviceNumber, &this->baseFrequency);
}

double FlinkPwm::get() {
	// TODO
	return 0;
}

void FlinkPwm::set(double dutyCycle) {
	this->setDutyCycle(dutyCycle);
}

void FlinkPwm::setFrequency(double f) {
	pwmFrequency = f;
	flink_pwm_set_period(deviceHandle, subDeviceNumber, channel, (uint32_t)(baseFrequency / pwmFrequency));
}

void FlinkPwm::setDutyCycle(double d) {
	if(d >= 0 && d <= 1) {
		flink_pwm_set_hightime(deviceHandle, subDeviceNumber, channel, (uint32_t)(baseFrequency / pwmFrequency * d));
	}
}
