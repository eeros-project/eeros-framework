#ifndef ORG_EEROS_CONTROL_BLOCK1O_HPP_
#define ORG_EEROS_CONTROL_BLOCK1O_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Signal.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class Block1o : public Block {
		public:
			Block1o() {
				this->out.getSignal().clear();
			}
			
			virtual Output<T>& getOut() {
				return out;
			}
			
		protected:
			Output<T> out;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_BLOCK1O_HPP_ */
