#ifndef ORG_EEROS_CONTROL_INPUT_HPP_
#define ORG_EEROS_CONTROL_INPUT_HPP_
#include <eeros/control/Output.hpp>
#include <eeros/control/SignalInterface.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class Input {
		public:
			Input() : connectedOutput(nullptr) { }

			virtual bool connect(Output<T>& output) {
				if(connectedOutput != nullptr) return false;
				connectedOutput = &output;
				return true;
			}
			
			virtual bool connect(Output<T>* output) {
				if(connectedOutput != nullptr) return false;
				connectedOutput = output;
				return true;
			}

			virtual void disconnect() {
				connectedOutput = nullptr;
			}

			virtual bool isConnected() const {
				return connectedOutput != nullptr;
			}
			
			virtual Signal<T>& getSignal() {
				if(isConnected()) return connectedOutput->getSignal();
				// TODO error message or throw exception
				return Signal<T>::getIllegalSignal();
			}
			
		protected:
			Output<T>* connectedOutput;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_INPUT_HPP_ */
