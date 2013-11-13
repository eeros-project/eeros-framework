#include <eeros/hal/ComediFqd.hpp>

using namespace eeros::hal;

ComediFqd::ComediFqd(std::string id, ComediDevice* device, uint32_t subDeviceNumber, uint32_t channelA, uint32_t channelB, uint32_t channelZ, double scale, double offset, double initValue) : SystemInput<double>(id) {
	this->deviceHandle = device->getDeviceHandle();
	this->subDeviceNumber = subDeviceNumber;
	this->channelA = channelA;
	this->channelB = channelB;
	this->channelZ = channelZ;
	this->scale = scale;
	this->offset = offset;
	
	comedi_reset(deviceHandle, subDeviceNumber);
	
	unsigned int devInitValue = 0; // TODO
	comedi_data_write(deviceHandle, subDeviceNumber, 0, 0, 0, devInitValue); // set initial counter value by writing to channel 0
	comedi_data_write(deviceHandle, subDeviceNumber, 1, 0, 0, devInitValue); // set "load a" register to initial_value by writing to channel 1
	comedi_data_write(deviceHandle, subDeviceNumber, 2, 0, 0, devInitValue); // set "load b" register to initial_value by writing to channel 2
	
	comedi_set_gate_source(deviceHandle, subDeviceNumber, 0, 0, NI_GPCT_DISABLED_GATE_SELECT);
    comedi_set_gate_source(deviceHandle, subDeviceNumber, 0, 1, NI_GPCT_DISABLED_GATE_SELECT);
	
	comedi_set_other_source(deviceHandle, subDeviceNumber, 0, NI_GPCT_SOURCE_ENCODER_A, NI_GPCT_PFI_OTHER_SELECT(channelA));
    comedi_set_other_source(deviceHandle, subDeviceNumber, 0, NI_GPCT_SOURCE_ENCODER_B, NI_GPCT_PFI_OTHER_SELECT(channelB));
    comedi_set_other_source(deviceHandle, subDeviceNumber, 0, NI_GPCT_SOURCE_ENCODER_Z, NI_GPCT_PFI_OTHER_SELECT(channelZ));
	
	lsampl_t counterMode;
	counterMode = (NI_GPCT_COUNTING_MODE_QUADRATURE_X4_BITS | NI_GPCT_COUNTING_DIRECTION_HW_UP_DOWN_BITS);
    if (channelZ != NI_GPCT_DISABLED_GATE_SELECT) counterMode |= (NI_GPCT_INDEX_ENABLE_BIT | NI_GPCT_INDEX_PHASE_HIGH_A_HIGH_B_BITS);
	comedi_set_counter_mode(deviceHandle, subDeviceNumber, 0, counterMode);
	
	comedi_arm(deviceHandle, subDeviceNumber, NI_GPCT_ARM_IMMEDIATE);
}

double ComediFqd::get() {
	lsampl_t data = 0;
	comedi_data_read(deviceHandle, subDeviceNumber, 0, 0, 0, &data);
	return static_cast<int>(data) * scale + offset;
}