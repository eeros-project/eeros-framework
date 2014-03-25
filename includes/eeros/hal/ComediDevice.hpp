#ifndef ORG_EEROS_HAL_COMEDIDEVICE_HPP_
#define ORG_EEROS_HAL_COMEDIDEVICE_HPP_

#include <string>

#include <comedilib.h>

#include <eeros/hal/SystemInput.hpp>
#include <eeros/hal/SystemOutput.hpp>

namespace eeros {
	namespace hal {

		class ComediDevice {
		public:
			ComediDevice(std::string deviceNode);
			virtual ~ComediDevice();
			
			comedi_t* getDeviceHandle();

		private:
			comedi_t *it;
		};

	};
};

#endif /* ORG_EEROS_HAL_COMEDIDEVICE_HPP_ */
