#ifndef ORG_EEROS_CONTROL_GATE_HPP_
#define ORG_EEROS_CONTROL_GATE_HPP_

#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class Gate : public Input<T>, public Output<T> {
		public:
			Gate() { }
			
			virtual Signal<T>& getSignal() {
				if(this->isConnected()) return this->connectedOutput->getSignal();
				// TODO error message or throw exception
				return Signal<T>::getIllegalSignal();
			}
		};
	};
};

#endif /* ORG_EEROS_CONTROL_GATE_HPP_ */
