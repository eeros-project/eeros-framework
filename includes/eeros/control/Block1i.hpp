#ifndef ORG_EEROS_CONTROL_BLOCK1I_HPP_
#define ORG_EEROS_CONTROL_BLOCK1I_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/RealSignalInput.hpp>

namespace eeros {
	namespace control {

		class Block1i: public Block {
		public:
			Block1i();
			RealSignalInput& getIn();
			
		protected:
			RealSignalInput in;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_BLOCK1I_HPP_ */
