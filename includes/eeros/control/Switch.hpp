#ifndef ORG_EEROS_CONTROL_SWITCH_HPP_
#define ORG_EEROS_CONTROL_SWITCH_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

namespace eeros {
	namespace control {

		template < uint8_t N = 2, typename T = double >
		class Switch : public Block {
		public:
			Switch(uint8_t initInputIndex) : currentInput(initInputIndex) { }
			
			virtual void run() {
				this->out.getSignal().setValue(this->in[currentInput].getSignal().getValue());
				this->out.getSignal().setTimestamp(this->in[currentInput].getSignal().getTimestamp());
			}
			
			virtual Input<T>& getIn(uint8_t index) {
				return in[index];
			}
			
			virtual Output<T>& getOut() {
				return out;
			}
			
			virtual bool switchToInput(uint8_t index) {
				if(index >= 0 && index < N) {
					currentInput = index;
					return true;
				}
				return false;
			}
			
		protected:
			Input<T> in[N];
			Output<T> out;
			uint8_t currentInput;
		};
	};
};

#endif /* ORG_EEROS_CONTROL_SWITCH_HPP_ */
