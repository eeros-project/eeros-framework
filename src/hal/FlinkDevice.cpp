#include <eeros/hal/FlinkDevice.hpp>
#include <eeros/core/EEROSException.hpp>

using namespace eeros::hal;

FlinkDevice::FlinkDevice(std::string deviceNode) {
	it = flink_open(deviceNode.c_str());
	if(!it) {
		throw EEROSException("Can't open device \"" +  deviceNode + "\"!");
	}
}

FlinkDevice::~FlinkDevice() {
	flink_close(it);
}

flink_dev* FlinkDevice::getDeviceHandle() {
	return it;
}
