#ifndef ORG_EEROS_CONTROL_PERIPHERALINPUT_HPP
#define ORG_EEROS_CONTROL_PERIPHERALINPUT_HPP

#include <eeros/control/Block1o.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/PeripheralInput.hpp>
#include <eeros/core/EEROSException.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class PeripheralInput : public Block1o<T> {

		public:
			PeripheralInput(std::string id) : hal(hal::HAL::instance()) {
				this->out.getSignal().clear();
				systemInput = dynamic_cast<eeros::hal::PeripheralInput<T>*>(hal.getPeripheralInput(id));
				if(systemInput == nullptr) throw EEROSException("Peripheral input '" + id + "' not found!");
			}
			
			virtual void run() {
				this->out.getSignal().setValue(systemInput->get());
				this->out.getSignal().setTimestamp(System::getTimeNs());
			}

		private:
			hal::HAL& hal;
			hal::PeripheralInput<T>* systemInput;
		};

	};
};

#endif // ORG_EEROS_CONTROL_PERIPHERALINPUT_HPP
