#include <eeros/hal/FlinkDigIn.hpp>

using namespace eeros::hal;

FlinkDigIn::FlinkDigIn(std::string id, FlinkDevice* device, uint8_t subDeviceNumber, uint32_t channel, bool inverted) : Input<bool>(id), channel(channel), inverted(inverted) {
	this->subdeviceHandle = flink_get_subdevice_by_id(device->getDeviceHandle(), subDeviceNumber);
	
	flink_dio_set_direction(subdeviceHandle, channel, FLINK_INPUT);
}

bool FlinkDigIn::get() {
	uint8_t data = 0;
	bool value;
	flink_dio_get_value(subdeviceHandle, channel, &data);
	value = static_cast<bool>(data);
	if(inverted) value = !value;
	return value;
}
