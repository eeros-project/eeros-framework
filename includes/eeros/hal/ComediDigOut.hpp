#ifndef ORG_EEROS_HAL_COMEDIDIGOUT_HPP_
#define ORG_EEROS_HAL_COMEDIDIGOUT_HPP_

#include <string>
#include <comedilib.h>
#include <eeros/hal/PeripheralOutput.hpp>
#include <eeros/hal/ComediDevice.hpp>

namespace eeros {
	namespace hal {

		class ComediDigOut : public PeripheralOutput<bool> {
		public:
			ComediDigOut(std::string id, ComediDevice* device, uint32_t subDeviceNumber, uint32_t channel, bool inverted = false);
			virtual bool get() const;
			virtual void set(bool value);
			
		private:
			comedi_t* deviceHandle;
			uint32_t subDeviceNumber;
			uint32_t channel;
			
			bool inverted;
		};

	};
};

#endif /* ORG_EEROS_HAL_COMEDIDIGOUT_HPP_ */
