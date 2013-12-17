#ifndef ORG_EEROS_HAL_COMEDIDAC_HPP_
#define ORG_EEROS_HAL_COMEDIDAC_HPP_

#include <string>
#include <vector>

#include <comedilib.h>

#include <eeros/hal/SystemOutput.hpp>
#include <eeros/hal/ComediDevice.hpp>

namespace eeros {
	namespace hal {

		class ComediDac : public SystemOutput<double> {
		public:
			ComediDac(std::string id, ComediDevice* device, uint32_t subDeviceNumber, uint32_t channel, double scale = 1, double offset = 0);
			virtual double get();
			virtual void set(double value);
			
			void setVoltageRange(double minVoltage, double maxVoltage);
			void setDacResolution(uint8_t bits);
			
		private:
			comedi_t* deviceHandle;
			uint32_t subDeviceNumber;
			uint32_t channel;
			
			double maxVoltage;
			double minVoltage;
			
			uint32_t maxValue;
			
			double scale;
			double offset;
		};

	};
};

#endif /* ORG_EEROS_HAL_COMEDIDAC_HPP_ */