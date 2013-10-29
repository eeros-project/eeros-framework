#ifndef ORG_EEROS_HAL_COMEDIADC_HPP_
#define ORG_EEROS_HAL_COMEDIADC_HPP_

#include <string>
#include <vector>

#include <comedilib.h>

#include <eeros/hal/SystemInput.hpp>
#include <eeros/hal/SystemOutput.hpp>
#include <eeros/hal/ComediDevice.hpp>

class ComediAdc : public SystemInput<double> {
public:
	ComediAdc(std::string id, ComediDevice& device, uint32_t subDeviceNumber, uint32_t channel, double scale, double offset);
	virtual double get();
	
private:
	comedi_t* deviceHandle;
	uint32_t subDeviceNumber;
	uint32_t channel;
	
	double scale;
	double offset;
};

#endif /* ORG_EEROS_HAL_COMEDIADC_HPP_ */