#ifndef ORG_EEROS_CONTROL_INPUT_HPP_
#define ORG_EEROS_CONTROL_INPUT_HPP_

#include <eeros/control/NotConnectedFault.hpp>
#include <eeros/control/Signal.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Block.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class Input {
		public:
			Input() : connectedOutput(nullptr), owner(nullptr) { }
			Input(Block* owner) : connectedOutput(nullptr), owner(owner) { }

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
				std::string name;
				if (owner != nullptr) name = owner->getName(); else name = "";
				throw NotConnectedFault("Read from an unconnected input in block '" + name + "'");
			}
			
			virtual void setOwner(Block* block) {
				owner = block;
			}
			
		protected:
			Output<T>* connectedOutput;
			Block* owner;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_INPUT_HPP_ */
