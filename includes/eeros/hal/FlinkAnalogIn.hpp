#ifndef ORG_EEROS_HAL_FLINKANALOGIN_HPP_
#define ORG_EEROS_HAL_FLINKANALOGIN_HPP_

#include <string>
#include <flinklib.h>
#include <eeros/hal/ScalablePeripheralInput.hpp>
#include <eeros/hal/FlinkDevice.hpp>

namespace eeros {
	namespace hal {

		class FlinkAnalogIn : public ScalablePeripheralInput<double> {
		public:
			FlinkAnalogIn(std::string id, FlinkDevice* device, uint8_t subDeviceNumber, uint32_t channel, double umax = 10, double umin = -10);
			virtual double get();
			
		private:
			flink_subdev* subdeviceHandle;
			uint32_t channel;
			uint32_t bitMask;
		};

	};
};

#endif /* ORG_EEROS_HAL_FLINKANALOGIN_HPP_ */
