#ifndef ORG_EEROS_HAL_FLINKWATCHDOG_HPP_
#define ORG_EEROS_HAL_FLINKWATCHDOG_HPP_

#include <string>
#include <flinklib.h>
#include <eeros/hal/PeripheralOutput.hpp>
#include <eeros/hal/FlinkDevice.hpp>

namespace eeros {
	namespace hal {

		class FlinkWatchdog : public PeripheralOutput<bool> {
		public:
			FlinkWatchdog(std::string id, FlinkDevice* device, uint32_t subDeviceNumber, double timeout = 0.05);
			
			virtual bool get();
			virtual void set(bool b);
			virtual void setTimeout(double t);
			virtual void reset();
			
		private:
			flink_subdev* subdeviceHandle;
			uint32_t channel;
			uint32_t baseClock;
			uint32_t counter;
		};
	};
};

#endif /* ORG_EEROS_HAL_FLINKWATCHDOG_HPP_ */
