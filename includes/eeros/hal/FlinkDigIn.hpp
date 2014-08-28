#ifndef ORG_EEROS_HAL_FLINKDIGIN_HPP_
#define ORG_EEROS_HAL_FLINKDIGIN_HPP_

#include <string>
#include <flinklib.h>
#include <eeros/hal/PeripheralInput.hpp>
#include <eeros/hal/FlinkDevice.hpp>

namespace eeros {
	namespace hal {

		class FlinkDigIn : public PeripheralInput<bool> {
		public:
			FlinkDigIn(std::string id, FlinkDevice* device, uint32_t subDeviceNumber, uint32_t channel, bool inverted = false);
			virtual bool get();
			virtual void set(bool value);
			
		private:
			flink_t* deviceHandle;
			uint32_t subDeviceNumber;
			uint32_t channel;
			
			bool inverted;
		};

	};
};

#endif /* ORG_EEROS_HAL_FLINKDIGIN_HPP_ */
