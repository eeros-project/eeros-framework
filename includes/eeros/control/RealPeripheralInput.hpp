#ifndef ORG_EEROS_CONTROL_REALPERIPHERALINPUT_HPP
#define ORG_EEROS_CONTROL_REALPERIPHERALINPUT_HPP

#include <eeros/control/Block1o.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/SystemInput.hpp>

namespace eeros {
	namespace control {

		class RealPeripheralInput : public Block1o {

		public:
			RealPeripheralInput(std::string outputId);
			virtual void run();

		private:
			hal::HAL& hal;
			hal::SystemInput<double>* systemInput;
		};

	};
};

#endif // ORG_EEROS_CONTROL_REALPERIPHERALINPUT_HPP
