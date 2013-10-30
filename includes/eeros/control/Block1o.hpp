#ifndef ORG_EEROS_CONTROL_BLOCK1O_HPP_
#define ORG_EEROS_CONTROL_BLOCK1O_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/RealSignalOutput.hpp>

namespace eeros {
	namespace control {

		class Block1o: public Block {
		public:
			Block1o(sigdim_t dim);
			
			RealSignalOutput& getOut();
			
		protected:
			RealSignalOutput out;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_BLOCK1O_HPP_ */
