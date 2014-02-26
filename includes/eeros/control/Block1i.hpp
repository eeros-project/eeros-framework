#ifndef ORG_EEROS_CONTROL_BLOCK1I_HPP_
#define ORG_EEROS_CONTROL_BLOCK1I_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Signal.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class Block1i : public Block {
		public:
			Block1i() { }
			
			virtual Input<T>& getIn() {
				return in;
			}
		
		protected:
			Input<T> in;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_BLOCK1I_HPP_ */
