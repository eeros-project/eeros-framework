#ifndef ORG_EEROS_HAL_FLINKANALOGOUT_HPP_
#define ORG_EEROS_HAL_FLINKANALOGOUT_HPP_

#include <string>
#include <flinklib.h>
#include <eeros/hal/ScalablePeripheralOutput.hpp>
#include <eeros/hal/FlinkDevice.hpp>

namespace eeros {
	namespace hal {

		class FlinkAnalogOut : public ScalablePeripheralOutput<double> {
		public:
			FlinkAnalogOut(std::string id, FlinkDevice* device, uint8_t subDeviceNumber, uint32_t channel, double scale = 1, double offset = 0);
			virtual void set(double voltage);
			virtual double get();
			void setValue(uint32_t value);
			
		private:
			flink_subdev* subdeviceHandle;
			uint32_t channel;
			uint32_t bitMask;
			uint32_t resolution;
		};

	};
};

#endif /* ORG_EEROS_HAL_FLINKANALOGOUT_HPP_ */
