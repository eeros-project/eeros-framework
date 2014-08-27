#include <eeros/hal/FlinkFqd.hpp>

using namespace eeros::hal;

FlinkFqd::FlinkFqd(std::string id, 
					 FlinkDevice* device,
					 uint32_t subDeviceNumber,
					 uint32_t counter,
					 double scale,
					 double offset,
					 double initValue) : 
					 ScalablePeripheralInput<double>(id, scale, offset) {
	this->deviceHandle = device->getDeviceHandle();
	this->subDeviceNumber = subDeviceNumber;
	this->channel = counter;
	this->prevPos = 0;
	reset();
}

double FlinkFqd::get() {
	uint32_t data = 0;
	flink_counter_get_count(deviceHandle, subDeviceNumber, channel, &data);
	int16_t newPos = static_cast<uint16_t>(data);
	int16_t delta = newPos - prevPos;
	prevPos = newPos;
	pos += delta * scale + offset;
	return pos;
}

void FlinkFqd::reset() {
	flink_subdevice_reset(deviceHandle, subDeviceNumber); // TODO only reset counter, not the subdevice!
	pos = 0;
}
