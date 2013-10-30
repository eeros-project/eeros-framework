#include "ComediEncoder.hpp"

using namespace eeros::examples::simpleMotorController;

ComediEncoder::ComediEncoder() : Block1o(1) {
    deviceName = ENCODER_DEV;
    subDevice = ENCODER_SUBDEV;
    it = comedi_open(deviceName.c_str());
    if(it == NULL) {
        comedi_perror("comedi_open");
    }
    ni_gpct_start_encoder(it, subDevice, 0, 8, 10, 9);
}


ComediEncoder::~ComediEncoder() {
    
}

void ComediEncoder::run() {
    comedi_data_read(it, subDevice, 0, 0, 0, &data);
    out.setTimeStamp(System::getTimeNs());
    out.setValue(static_cast<int>(data) * 6.28318530718 / 2000.0);
//	out.setValue(static_cast<double>(data));
}

int ComediEncoder::ni_gpct_start_encoder(comedi_t *device, unsigned subdevice, unsigned int initial_value, int a, int b, int z) {
    int retval;
    lsampl_t counter_mode;

    retval = comedi_reset(device, subdevice);

    /* set initial counter value by writing to channel 0 */
    retval = comedi_data_write(device, subdevice, 0, 0, 0, initial_value);
    
    /* set "load a" register to initial_value by writing to channel 1 */
    retval = comedi_data_write(device, subdevice, 1, 0, 0, initial_value);
    
    /* set "load b" register to initial_value by writing to channel 2 */
    retval = comedi_data_write(device, subdevice, 2, 0, 0, initial_value);

    comedi_set_gate_source(device, subdevice, 0, 0, NI_GPCT_DISABLED_GATE_SELECT);
    comedi_set_gate_source(device, subdevice, 0, 1, NI_GPCT_DISABLED_GATE_SELECT);
    /* note, the comedi_set_other_source calls will fail on 660x boards, since they
     * don't support user selection of the inputs used for the A/B/Z signals. */
    comedi_set_other_source(device, subdevice, 0, NI_GPCT_SOURCE_ENCODER_A, NI_GPCT_PFI_OTHER_SELECT(a));
    comedi_set_other_source(device, subdevice, 0, NI_GPCT_SOURCE_ENCODER_B, NI_GPCT_PFI_OTHER_SELECT(b));
    comedi_set_other_source(device, subdevice, 0, NI_GPCT_SOURCE_ENCODER_Z, NI_GPCT_PFI_OTHER_SELECT(z));

    counter_mode = (NI_GPCT_COUNTING_MODE_QUADRATURE_X4_BITS |
        NI_GPCT_COUNTING_DIRECTION_HW_UP_DOWN_BITS);
    if (z != NI_GPCT_DISABLED_GATE_SELECT) {
        counter_mode |= (NI_GPCT_INDEX_ENABLE_BIT |
            NI_GPCT_INDEX_PHASE_HIGH_A_HIGH_B_BITS);
    }
    retval = comedi_set_counter_mode(device, subdevice, 0, counter_mode);
    if(retval < 0) return retval;

    retval = comedi_arm(device, subdevice, NI_GPCT_ARM_IMMEDIATE);
    if(retval < 0) return retval;

    return 0;
}
