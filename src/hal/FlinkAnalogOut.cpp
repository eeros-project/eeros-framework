#include <eeros/hal/FlinkAnalogOut.hpp>

using namespace eeros::hal;

FlinkAnalogOut::FlinkAnalogOut(std::string id, FlinkDevice* device, uint8_t subDeviceNumber, uint32_t channel, double scale, double offset) : ScalablePeripheralOutput<double>(id, scale, offset), channel(channel), bitMask(0) {
	this->subdeviceHandle = flink_get_subdevice_by_id(device->getDeviceHandle(), subDeviceNumber);

	flink_analog_out_get_resolution(subdeviceHandle, &resolution);
	bitMask = (1 << resolution) - 1;
	
	this->scale = scale;
	this->offset = offset;
}

double FlinkAnalogOut::get() {
	// TODO
	return 0;
}

void FlinkAnalogOut::setValue(uint32_t value) {
	value &= bitMask;
	flink_analog_out_set_value(subdeviceHandle, channel, value);
}

void FlinkAnalogOut::set(double voltage) {
	uint32_t value = static_cast<uint32_t>((voltage - offset)/scale);
	setValue(value);
}



