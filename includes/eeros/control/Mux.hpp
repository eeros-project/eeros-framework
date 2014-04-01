#ifndef ORG_EEROS_CONTROL_MUX_HPP_
#define ORG_EEROS_CONTROL_MUX_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

namespace eeros {
	namespace control {
		
		template < uint32_t N, typename T = double, typename C = eeros::math::Matrix<N,1,T> >
		class Mux: public Block {
			
		public:
			Mux() { }
			
			virtual void run() {
				C newValue;
				for(int i = 0; i < N; i++) {
					newValue(i) = in[i].getSignal().getValue();
				}
				out.getSignal().setValue(newValue);
				out.getSignal().setTimestamp(in[0].getSignal().getTimestamp());
			}
			
			virtual Input<T>& getIn(uint32_t index) {
				return in[index];
			}
			
			virtual Output<C>& getOut() {
				return out;
			}
			
		protected:
			Input<T> in[N];
			Output<C> out;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_MUX_HPP_ */
