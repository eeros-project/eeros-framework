#include <eeros/hal/FlinkPwm.hpp>

using namespace eeros::hal;

FlinkPwm::FlinkPwm(std::string id,
				   FlinkDevice* device,
				   uint32_t subDeviceNumber,
				   uint32_t channel,
				   double scale,
				   double offset) : ScalablePeripheralOutput<double>(id, scale, offset), channel(channel) {
	subdeviceHandle = flink_get_subdevice_by_id(device->getDeviceHandle(), subDeviceNumber);
	flink_pwm_get_baseclock(subdeviceHandle, &baseFrequency);
}

double FlinkPwm::get() {
	// TODO
	return 0;
}

void FlinkPwm::set(double dutyCycle) {
	setDutyCycle(dutyCycle);
}

void FlinkPwm::setFrequency(double f) {
	pwmFrequency = f;
	flink_pwm_set_period(subdeviceHandle, channel, (uint32_t)(baseFrequency / pwmFrequency));
}

void FlinkPwm::setDutyCycle(double d) {
	if(d >= 0 && d <= 1) {
		flink_pwm_set_hightime(subdeviceHandle, channel, (uint32_t)(baseFrequency / pwmFrequency * d));
	}
}
