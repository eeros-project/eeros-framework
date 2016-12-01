#ifndef ORG_EEROS_HAL_FLINKDIGIN_HPP_
#define ORG_EEROS_HAL_FLINKDIGIN_HPP_

#include <string>
#include <flinklib.h>
#include <eeros/hal/Input.hpp>
#include <eeros/hal/FlinkDevice.hpp>

namespace eeros {
	namespace hal {

		class FlinkDigIn : public Input<bool> {
		public:
			FlinkDigIn(std::string id, FlinkDevice* device, uint8_t subDeviceNumber, uint32_t channel, bool inverted = false);
			virtual bool get();
			
		private:
			flink_subdev* subdeviceHandle;
			uint32_t channel;
			bool inverted;
		};

	};
};

#endif /* ORG_EEROS_HAL_FLINKDIGIN_HPP_ */
