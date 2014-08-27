#include <eeros/hal/FlinkAnalogIn.hpp>

using namespace eeros::hal;

FlinkAnalogIn::FlinkAnalogIn(std::string id, FlinkDevice* device, uint32_t subDeviceNumber, uint32_t channel) : PeripheralOutput<double>(id) {
	this->deviceHandle = device->getDeviceHandle();
	this->subDeviceNumber = subDeviceNumber;
	this->channel = channel;
	this->bit_mask = 0;
	//cache resolution	
	flink_analog_in_get_resolution(deviceHandle,subDeviceNumber,&resolution);

	for(uint32_t i = 0; i < resolution; i++){
		bit_mask = bit_mask | (0x1 << i);
	}

}

double FlinkAnalogIn::get() {
	uint32_t data = 0;
	flink_analog_in_get_value(deviceHandle,subDeviceNumber,channel,&data);
	data = data & bit_mask;
	return data;
}

void FlinkAnalogIn::set(double value) {
	//do nothing
}
