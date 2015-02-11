#ifndef ORG_EEROS_HAL_FLINKFQD_HPP_
#define ORG_EEROS_HAL_FLINKFQD_HPP_

#include <string>
#include <flinklib.h>
#include <eeros/hal/ScalablePeripheralInput.hpp>
#include <eeros/hal/FlinkDevice.hpp>

namespace eeros {
	namespace hal {
		
		class FlinkFqd : public ScalablePeripheralInput<double> {
		public:
			FlinkFqd(std::string id, FlinkDevice* device, uint32_t subDeviceNumber, uint32_t channel, double scale = 1, double offset = 0, bool getDelta = false);
			virtual double get();
			void reset();
			
		private:
			flink_subdev* subdeviceHandle;
			uint32_t channel;
			uint16_t prevPos;
			double pos;
			bool getDelta;
		};
	};
};

#endif /* ORG_EEROS_HAL_FLINKFQD_HPP_ */
