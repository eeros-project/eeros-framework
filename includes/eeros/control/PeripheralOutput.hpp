#ifndef ORG_EEROS_CONTROL_PERIPHERALOUTPUT_HPP
#define ORG_EEROS_CONTROL_PERIPHERALOUTPUT_HPP

#include <cmath>
#include <eeros/control/Block1i.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/control/NaNOutputFault.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class PeripheralOutput : public Block1i<T> {

		public:
			PeripheralOutput(std::string id) : hal(hal::HAL::instance()) {
				systemOutput = dynamic_cast<hal::PeripheralOutput<T>*>(hal.getPeripheralOutput(id));
				if(systemOutput == nullptr) throw Fault("Peripheral output '" + id + "' not found!");
			}
			
			virtual void run() {
				T val = this->in.getSignal().getValue();
				if(isnan(val)) throw NaNOutputFault("NaN written to output");
				systemOutput->set(this->in.getSignal().getValue());
			}
			
		private:
			hal::HAL& hal;
			hal::PeripheralOutput<T>* systemOutput;
		};

	};
};

#endif // ORG_EEROS_CONTROL_PERIPHERALOUTPUT_HPP
