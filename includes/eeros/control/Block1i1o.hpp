#ifndef ORG_EEROS_CONTROL_BLOCK1I1O_HPP_
#define ORG_EEROS_CONTROL_BLOCK1I1O_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class Block1i1o : public Block {
		public:
			Block1i1o() { }
			
			virtual Input<T>& getIn() {
				return in;
			}
			
			virtual Output<T>& getOut() {
				return out;
			}
			
		protected:
			Input<T> in;
			Output<T> out;
		};
		
	};
};
#endif /* ORG_EEROS_CONTROL_BLOCK1I1O_HPP_ */
