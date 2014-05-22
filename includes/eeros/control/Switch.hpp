#ifndef ORG_EEROS_CONTROL_SWITCH_HPP_
#define ORG_EEROS_CONTROL_SWITCH_HPP_

#include <vector>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

namespace eeros {
	namespace control {

		template < uint8_t N = 2, typename T = double >
		class Switch : public Block {
		public:
			Switch(uint8_t initInputIndex) : currentInput(initInputIndex), maxDeviation(defaultDeviation) { }
			
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
			
			/**
			 * @brief Set max. difference between the input signals which is allowed for switching the input.
			 * This means if you want to switch from input 0 to input 1, the value on input 1 (val1) has to be
			 * somewere in the range of val0 - (val0 * maxDifference) to val0 + (val0 * maxDifference).
			 *
			 * @param maxDifference max. difference (relative value).
			 * @return void
			 **/
			virtual void setSwitchTolerance(double maxDeviation) {
				this->maxDeviation = maxDeviation;
			}
			
			
			virtual bool switchToInput(uint8_t index) {
				bool checkOK = true;
				T inValC, inValN;
				uint8_t i = 0;
				while(i < N && checkOK) {
					inValC = this->in[currentInput].getSignal().getValue();
					inValN = this->in[index].getSignal().getValue();
					checkOK = (inValN < inValC + maxDeviation * inValC) && (inValN > inValC - maxDeviation * inValC);
					i++;
				}
				if(checkOK) {
					currentInput = index;
					return true;
				}
				return false;
			}
			
		protected:
			Input<T> in[N];
			Output<T> out;
			uint8_t currentInput;
			double maxDeviation;
			const double defaultDeviation = 0.1; // 10%
		};
	};
};

#endif /* ORG_EEROS_CONTROL_SWITCH_HPP_ */
