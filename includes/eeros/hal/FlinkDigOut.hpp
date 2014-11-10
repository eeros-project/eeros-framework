#ifndef ORG_EEROS_HAL_FLINKDIGOUT_HPP_
#define ORG_EEROS_HAL_FLINKDIGOUT_HPP_

#include <string>
#include <flinklib.h>
#include <eeros/hal/PeripheralOutput.hpp>
#include <eeros/hal/FlinkDevice.hpp>

namespace eeros {
	namespace hal {

		class FlinkDigOut : public PeripheralOutput<bool> {
		public:
			FlinkDigOut(std::string id, FlinkDevice* device, uint32_t subDeviceNumber, uint32_t channel, bool inverted = false);
			virtual bool get();
			virtual void set(bool value);
			
		private:
			flink_subdev* subdeviceHandle;
			uint32_t channel;
			bool inverted;
		};

	};
};

#endif /* ORG_EEROS_HAL_FLINKDIGOUT_HPP_ */
