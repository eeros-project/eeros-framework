#ifndef ORG_EEROS_HAL_COMEDIADC_HPP_
#define ORG_EEROS_HAL_COMEDIADC_HPP_

#include <string>
#include <comedilib.h>
#include <eeros/hal/ScalablePeripheralInput.hpp>
#include <eeros/hal/ComediDevice.hpp>

namespace eeros {
	namespace hal {

		class ComediAdc : public ScalablePeripheralInput<double> {
		public:
			ComediAdc(std::string id, ComediDevice* device, uint32_t subDeviceNumber, uint32_t channel, double scale = 1, double offset = 0);
			virtual double get() const;
			
		private:
			comedi_t* deviceHandle;
			uint32_t subDeviceNumber;
			uint32_t channel;
		};

	};
};

#endif /* ORG_EEROS_HAL_COMEDIADC_HPP_ */
