#ifndef ORG_EEROS_HAL_FLINKPWM_HPP_
#define ORG_EEROS_HAL_FLINKPWM_HPP_

#include <string>
#include <flinklib.h>
#include <eeros/hal/ScalableOutput.hpp>
#include <eeros/hal/FlinkDevice.hpp>

namespace eeros {
	namespace hal {

		class FlinkPwm : public ScalableOutput<double> {
		public:
			FlinkPwm(std::string id, FlinkDevice* device, uint32_t subDeviceNumber, uint32_t channel, double scale = 1, double offset = 0);
			virtual double get();
			virtual void set(double dutyCycle);
			virtual void setFrequency(double f);
			virtual void setDutyCycle(double d);
			
		private:
			flink_subdev* subdeviceHandle;
			uint32_t channel;
			double pwmFrequency;
			uint32_t baseFrequency;
		};

	};
};

#endif /* ORG_EEROS_HAL_FLINKPWM_HPP_ */
