#ifndef ORG_EEROS_EXAMPLES_SIMPLEMOTORCONTROLER_COMEDIDAC_HPP_
#define ORG_EEROS_EXAMPLES_SIMPLEMOTORCONTROLER_COMEDIDAC_HPP_

#include <eeros/control/RealSignalOutput.hpp>
#include <eeros/control/Block1i.hpp>

#include <string>
#include <comedilib.h>

#define DAC_DEV "/dev/comedi0"
#define DAC_SUBDEV 1
#define DAC_DEFAULT_VAL 0

enum { minVal = 0, maxVal = 65535 };

class ComediDac : public Block1i
{

public:
    ComediDac(int channel);
    virtual ~ComediDac();
    virtual void run();

private:
    comedi_t* it;
    std::string deviceName;
    int subDevice;
    int channel;
};

#endif /* ORG_EEROS_EXAMPLES_SIMPLEMOTORCONTROLER_COMEDIDAC_HPP_ */
