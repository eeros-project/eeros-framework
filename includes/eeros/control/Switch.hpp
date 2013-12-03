#ifndef ORG_EEROS_CONTROL_SWITCH_HPP_
#define ORG_EEROS_CONTROL_SWITCH_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/RealSignalInput.hpp>
#include <eeros/control/RealSignalOutput.hpp>

#include <vector>

namespace eeros {
	namespace control {

		class Switch : public Block {
		public:
			Switch(uint8_t nofInputs = 2, sigdim_t dim = 1);

			virtual void run();
			
			virtual RealSignalInput& getIn(uint8_t input = 0);
			virtual RealSignalOutput& getOut();
			
			/**
			 * @brief Set max. difference between the input signals which is allowed for switching the input.
			 * This means if you want to switch from input 0 to input 1, the value on input 1 (val1) has to be
			 * somewere in the range of val0 - maxDifference to val0 + maxDifference. Use a negative Value
			 * (maxDifference < 0) to disable checking.
			 *
			 * @param maxDifference max. difference as absolute value.
			 * @return void
			 **/
			virtual void setSwitchTolerance(double maxDifference);
			
			virtual void setSwitchTolerance(std::vector<double> maxDifference);
			
			virtual bool switchToInput(uint8_t input);
			
		protected:
			std::vector<RealSignalInput> in;
			RealSignalOutput out;
			uint8_t currentInput;
			std::vector<double> maxDifference;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_SWITCH_HPP_ */
