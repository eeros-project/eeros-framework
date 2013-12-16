#ifndef ORG_EEROS_CONTROL_REALPERIPHERALINPUT_HPP
#define ORG_EEROS_CONTROL_REALPERIPHERALINPUT_HPP

#include <eeros/control/Block1o.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/SystemInput.hpp>

namespace eeros {
	namespace control {

		class RealPeripheralInput : public Block1o {

		public:
			RealPeripheralInput(std::string outputId, double scale = 1, double offset = 0);
			virtual void run();
			virtual void setOffset(double o);
			virtual void setScale(double s);

		private:
			hal::HAL& hal;
			hal::SystemInput<double>* systemInput;
			double scale;
			double offset;
		};

	};
};

#endif // ORG_EEROS_CONTROL_REALPERIPHERALINPUT_HPP
