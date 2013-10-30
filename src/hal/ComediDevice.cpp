#include <eeros/hal/ComediDevice.hpp>

using namespace eeros::hal;

ComediDevice::ComediDevice(std::string deviceNode) {
	it = comedi_open("/dev/comedi0");
	if(!it) {
		throw -1; // TODO
	}
}

ComediDevice::~ComediDevice() {
	comedi_close(it);
}

comedi_t* ComediDevice::getDeviceHandle() {
	return it;
}