#ifndef ORG_EEROS_CONTROL_MUX_HPP_
#define ORG_EEROS_CONTROL_MUX_HPP_

#include <vector>

#include <eeros/control/Block.hpp>
#include <eeros/control/RealSignalInput.hpp>
#include <eeros/control/RealSignalOutput.hpp>

namespace eeros {
	namespace control {

		class Mux: public Block {
		public:
			Mux(sigdim_t nofInputs);
			
			virtual void run();
			
			virtual RealSignalInput& getIn(uint8_t input = 0);
			virtual RealSignalOutput& getOut();
			
		protected:
			std::vector<RealSignalInput> in;
			RealSignalOutput out;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_MUX_HPP_ */
