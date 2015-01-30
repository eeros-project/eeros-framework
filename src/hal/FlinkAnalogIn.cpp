#include <eeros/hal/FlinkAnalogIn.hpp>

using namespace eeros::hal;

FlinkAnalogIn::FlinkAnalogIn(std::string id, FlinkDevice* device, uint8_t subDeviceNumber, uint32_t channel, double umax, double umin) : ScalablePeripheralInput<double>(id, 1, 0), channel(channel), bitMask(0) {
	this->subdeviceHandle = flink_get_subdevice_by_id(device->getDeviceHandle(), subDeviceNumber);
	
	uint32_t resolution;
	flink_analog_in_get_resolution(subdeviceHandle, &resolution);
	bitMask = (1 << resolution) - 1;
	
	scale = umax - umin / ((1 << resolution) - 1);
	offset = -(umax - umin) / 2;
}

double FlinkAnalogIn::get() {
	uint32_t data;
	flink_analog_in_get_value(subdeviceHandle, channel, &data);
	data &= bitMask;
	return data * scale + offset;
}
