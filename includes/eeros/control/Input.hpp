#ifndef ORG_EEROS_CONTROL_INPUT_HPP_
#define ORG_EEROS_CONTROL_INPUT_HPP_

#include <eeros/control/NotConnectedFault.hpp>
#include <eeros/control/Signal.hpp>
#include <eeros/control/Output.hpp>

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
				throw NotConnectedFault("Read from an unconnected input");
			}
			
		protected:
			Output<T>* connectedOutput;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_INPUT_HPP_ */
