#include <eeros/hal/FlinkDigIn.hpp>

using namespace eeros::hal;

FlinkDigIn::FlinkDigIn(std::string id, FlinkDevice* device, uint32_t subDeviceNumber, uint32_t channel, bool inverted) : PeripheralInput<bool>(id) {
	this->deviceHandle = device->getDeviceHandle();
	this->subDeviceNumber = subDeviceNumber;
	this->channel = channel;
	this->inverted = inverted;
	
	flink_dio_set_direction(deviceHandle, subDeviceNumber, channel, FLINK_INPUT);
}

bool FlinkDigIn::get() {
	uint8_t data = 0;
	bool value;
	flink_dio_get_value(deviceHandle, subDeviceNumber, channel, &data);
	value = static_cast<bool>(data);
	if(inverted) value = !value;
	return value;
}

void FlinkDigIn::set(bool value) {
	//do nothing
}
