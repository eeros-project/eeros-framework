#include <eeros/hal/FlinkAnalogIn.hpp>

using namespace eeros::hal;

FlinkAnalogIn::FlinkAnalogIn(std::string id, FlinkDevice* device, uint8_t subDeviceNumber, uint32_t channel, double scale, double offset, bool twosComplement) : ScalableInput<double>(id, 1, 0), channel(channel), bitMask(0) {
	this->subdeviceHandle = flink_get_subdevice_by_id(device->getDeviceHandle(), subDeviceNumber);

	flink_analog_in_get_resolution(subdeviceHandle, &resolution);
	bitMask = (1 << resolution) - 1;
	
	this->scale = scale;
	this->offset = offset;
	this->twosComplement = twosComplement;
}

double FlinkAnalogIn::get() {
	uint32_t data;
	flink_analog_in_get_value(subdeviceHandle, channel, &data);
	data &= bitMask;
	if(twosComplement && (data >> (resolution - 1)) >= 1){ //negative number
		int32_t complementsData = -((~data&bitMask) + 1); //invert bitwise and add one
		return complementsData  * scale / bitMask + offset;	
	}else{
		return data * scale / bitMask + offset;
	}
}
