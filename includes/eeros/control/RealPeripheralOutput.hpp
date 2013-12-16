#ifndef ORG_EEROS_CONTROL_REALPERIPHERALOUTPUT_HPP
#define ORG_EEROS_CONTROL_REALPERIPHERALOUTPUT_HPP

#include <eeros/control/Block1i.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/SystemOutput.hpp>

namespace eeros {
	namespace control {

		class RealPeripheralOutput : public Block1i {

		public:
			RealPeripheralOutput(std::string outputId, double scale = 1, double offset = 0);
			virtual void run();
			virtual void setOffset(double o);
			virtual void setScale(double s);
			
		private:
			hal::HAL& hal;
			hal::SystemOutput<double>* systemOutput;
			double scale;
			double offset;
		};

	};
};

#endif // ORG_EEROS_CONTROL_REALPERIPHERALOUTPUT_HPP
