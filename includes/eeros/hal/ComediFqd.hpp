#ifndef ORG_EEROS_HAL_COMEDIFQD_HPP_
#define ORG_EEROS_HAL_COMEDIFQD_HPP_

#include <string>
#include <vector>

#include <comedilib.h>

#include <eeros/hal/SystemInput.hpp>
#include <eeros/hal/SystemOutput.hpp>
#include <eeros/hal/ComediDevice.hpp>

namespace eeros {
	namespace hal {

		class ComediFqd : public SystemInput<double> {
		public:
			ComediFqd(std::string id, ComediDevice* device, uint32_t subDeviceNumber, uint32_t channelA, uint32_t channelB, uint32_t channelZ, double scale = 1, double offset = 0, double initValue = 0);
			virtual double get();
			
		private:
			comedi_t* deviceHandle;
			uint32_t subDeviceNumber;
			uint32_t channelA;
			uint32_t channelB;
			uint32_t channelZ;
			lsampl_t counter_mode;
			
			double scale;
			double offset;
		};

	};
};

#endif /* ORG_EEROS_HAL_COMEDIFQD_HPP_ */