#ifndef ORG_EEROS_HAL_FLINKANALOGIN_HPP_
#define ORG_EEROS_HAL_FLINKANALOGIN_HPP_

#include <string>
#include <flinklib.h>
#include <eeros/hal/PeripheralOutput.hpp>
#include <eeros/hal/FlinkDevice.hpp>

namespace eeros {
	namespace hal {

		class FlinkAnalogIn : public PeripheralOutput<double> {
		public:
			FlinkAnalogIn(std::string id, FlinkDevice* device, uint32_t subDeviceNumber, uint32_t channel);
			virtual double get();
			virtual void set(double value);
			
		private:
			flink_t* deviceHandle;
			uint32_t subDeviceNumber;
			uint32_t channel;
			uint32_t resolution;
			uint32_t bit_mask;
		};

	};
};

#endif /* ORG_EEROS_HAL_FLINKANALOGIN_HPP_ */
