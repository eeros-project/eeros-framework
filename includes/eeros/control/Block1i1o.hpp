#ifndef ORG_EEROS_CONTROL_BLOCK1I1O_HPP_
#define ORG_EEROS_CONTROL_BLOCK1I1O_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

namespace eeros {
	namespace control {

		template < typename Tin = double, typename Tout = Tin >
		class Block1i1o : public Block {
		public:
			Block1i1o() {
				this->out.getSignal().clear();
			}
			
			virtual Input<Tin>& getIn() {
				return in;
			}
			
			virtual Output<Tout>& getOut() {
				return out;
			}
			
		protected:
			Input<Tin> in;
			Output<Tout> out;
		};
		
	};
};
#endif /* ORG_EEROS_CONTROL_BLOCK1I1O_HPP_ */
