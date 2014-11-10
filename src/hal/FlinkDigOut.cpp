#include <eeros/hal/FlinkDigOut.hpp>

using namespace eeros::hal;

FlinkDigOut::FlinkDigOut(std::string id, FlinkDevice* device, uint32_t subDeviceNumber, uint32_t channel, bool inverted) : PeripheralOutput<bool>(id), channel(channel), inverted(inverted) {
	this->subdeviceHandle = flink_get_subdevice_by_id(device->getDeviceHandle(), subDeviceNumber);
	
	flink_dio_set_direction(subdeviceHandle, channel, FLINK_OUTPUT);
}

bool FlinkDigOut::get() {
	uint8_t data = 0;
	bool value;
	flink_dio_get_value(subdeviceHandle, channel, &data);
	value = static_cast<bool>(data);
	if(inverted) value = !value;
	return value;
}

void FlinkDigOut::set(bool value) {
	if(inverted) value = !value;
	flink_dio_set_value(subdeviceHandle, channel, value);
}
