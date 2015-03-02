#include <eeros/hal/FlinkWatchdog.hpp>
using namespace eeros::hal;

FlinkWatchdog::FlinkWatchdog(std::string id, FlinkDevice* device, uint32_t subDeviceNumber, double timeout) : PeripheralOutput<bool>(id), channel(channel) {
	subdeviceHandle = flink_get_subdevice_by_id(device->getDeviceHandle(), subDeviceNumber);
	flink_wd_get_baseclock(subdeviceHandle, &baseClock);
	setTimeout(timeout);
}

void FlinkWatchdog::set(bool b) {
	if (b)
		flink_wd_set_counter(subdeviceHandle, counter);
	else
		flink_wd_set_counter(subdeviceHandle, 0);
}

void FlinkWatchdog::setTimeout(double t) {
	counter = static_cast<uint32_t>(baseClock * t);
}

bool FlinkWatchdog::get() {
	uint8_t status;
	flink_wd_get_status(subdeviceHandle, &status);
	return static_cast<bool>(status);
}

void FlinkWatchdog::reset() {
	flink_wd_set_counter(subdeviceHandle, counter);
	flink_wd_arm(subdeviceHandle);
}
