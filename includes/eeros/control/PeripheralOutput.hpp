#ifndef ORG_EEROS_CONTROL_PERIPHERALOUTPUT_HPP
#define ORG_EEROS_CONTROL_PERIPHERALOUTPUT_HPP

#include <eeros/control/Block1i.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/SystemOutput.hpp>
#include <eeros/core/EEROSException.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class PeripheralOutput : public Block1i<T> {

		public:
			PeripheralOutput(std::string id) : hal(hal::HAL::instance()) {
				systemOutput = dynamic_cast<hal::SystemOutput<T>*>(hal.getSystemOutput(id));
				if(systemOutput == nullptr) throw EEROSException("System output '" + id + "' not found!");
			}
			
			virtual void run() {
				systemOutput->set(this->in.getSignal().getValue());
			}
			
		private:
			hal::HAL& hal;
			hal::SystemOutput<T>* systemOutput;
		};

	};
};

#endif // ORG_EEROS_CONTROL_PERIPHERALOUTPUT_HPP
