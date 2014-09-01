#include <eeros/hal/FlinkWatchdog.hpp>
using namespace eeros::hal;

FlinkWatchdog::FlinkWatchdog(std::string id, FlinkDevice* device, uint32_t subDeviceNumber, uint32_t channel) : PeripheralOutput<double>(id) {
	this->deviceHandle = device->getDeviceHandle();
	this->subDeviceNumber = subDeviceNumber;
	this->channel = channel;
	//cache base clock
	flink_wd_get_baseclock(deviceHandle,subDeviceNumber, &baseClock);	

}

double FlinkWatchdog::get() {
	//nothing to return	
	return 0;
}

void FlinkWatchdog::set(double frequency) {
	uint32_t value = uint32_t(baseClock/frequency);
	flink_wd_set_counter_reg(deviceHandle, subDeviceNumber, channel, value);
}

void FlinkWatchdog::reset(){
	flink_wd_reset_chanel(deviceHandle,subDeviceNumber,channel);
}
void FlinkWatchdog::setClkPol(bool pol){
	if(pol){
		flink_wd_set_clk_pol(deviceHandle,subDeviceNumber,channel,1);
	}else{
		flink_wd_set_clk_pol(deviceHandle,subDeviceNumber,channel,1);
	}
}
