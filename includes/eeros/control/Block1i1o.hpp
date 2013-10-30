#ifndef ORG_EEROS_CONTROL_BLOCK1I1O_HPP_
#define ORG_EEROS_CONTROL_BLOCK1I1O_HPP_

#include <eeros/types.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/RealSignalInput.hpp>
#include <eeros/control/RealSignalOutput.hpp>

namespace eeros {
	namespace control {

		class Block1i1o: public Block {
		public:
			Block1i1o(sigdim_t dim = 1);
			virtual RealSignalInput& getIn();
			virtual RealSignalOutput& getOut();
			
		protected:
			RealSignalInput in;
			RealSignalOutput out;
		};
		
	};
};
#endif /* ORG_EEROS_CONTROL_BLOCK1I1O_HPP_ */
