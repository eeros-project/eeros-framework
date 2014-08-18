#ifndef ORG_EEROS_HAL_FLINKWATCHDOG_HPP_
#define ORG_EEROS_HAL_FLINKWATCHDOG_HPP_

#include <string>
#include <flinklib.h>
#include <eeros/hal/PeripheralOutput.hpp>
#include <eeros/hal/FlinkDevice.hpp>

namespace eeros {
	namespace hal {

		class FlinkWatchdog : public PeripheralOutput<double> {
		public:
			FlinkWatchdog(std::string id, FlinkDevice* device, uint32_t subDeviceNumber, uint32_t channel);
			virtual double get();
			virtual void set(double frequency);
			virtual void reset();
			virtual void setClkPol(bool pol); //false == rising edge, true == falling edge			
		

		private:
			flink_t* deviceHandle;
			uint32_t subDeviceNumber;
			uint32_t channel;
			uint32_t baseClock;

		};

	};
};

#endif /* ORG_EEROS_HAL_FLINKWATCHDOG_HPP_ */
