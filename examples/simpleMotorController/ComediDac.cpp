#include "ComediDac.hpp"

using namespace eeros::examples::simpleMotorController;

ComediDac::ComediDac(int chan) {
    deviceName = DAC_DEV;
    subDevice = DAC_SUBDEV;
    channel = chan;
    
    it = comedi_open(deviceName.c_str());
    if(it == NULL) {
        comedi_perror("comedi_open");
    }
}

ComediDac::~ComediDac() {

}

void ComediDac::run() {
	double inputVal = in.getValue();
	if(inputVal > 10) inputVal = 10;
	if(inputVal < -10) inputVal = -10;
    lsampl_t data = static_cast<lsampl_t>(inputVal * maxVal / 20.0 + maxVal / 2.0);
    int retval = comedi_data_write(it, subDevice, channel, 0, AREF_GROUND, data);
    if(retval < 0) {
        comedi_perror("comedi_data_write");
    }
}