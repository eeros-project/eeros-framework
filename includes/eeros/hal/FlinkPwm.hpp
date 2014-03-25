#ifndef ORG_EEROS_HAL_FLINKPWM_HPP_
#define ORG_EEROS_HAL_FLINKPWM_HPP_

#include <string>
#include <flinklib.h>

#include <eeros/hal/SystemOutput.hpp>
#include <eeros/hal/FlinkDevice.hpp>

namespace eeros {
	namespace hal {

		class FlinkPwm : public SystemOutput<double> {
		public:
			FlinkPwm(std::string id, FlinkDevice* device, uint32_t subDeviceNumber, uint32_t channel, double scale = 1, double offset = 0);
			virtual void set(double frequency, double dutyCycle);
			
		private:
			flink_t* deviceHandle;
			uint32_t subDeviceNumber;
			uint32_t channel;
			
			double scale;
			double offset;
		};

	};
};

#endif /* ORG_EEROS_HAL_FLINKPWM_HPP_ */
