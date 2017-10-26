#ifndef ORG_EEROS_CONTROL_OUTPUT_HPP_
#define ORG_EEROS_CONTROL_OUTPUT_HPP_

#include <eeros/control/Signal.hpp>
#include <eeros/control/Block.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class Output {
		public:
			Output() : owner(nullptr) { }
			Output(Block* owner) : owner(owner) { }
			
			virtual Signal<T>& getSignal() {
				return signal;
			}
			
			virtual void setOwner(Block* block) {
				owner = block;
			}
			
		private:
			Signal<T> signal;
			Block* owner;
		};
	};
};

#endif /* ORG_EEROS_CONTROL_OUTPUT_HPP_ */
