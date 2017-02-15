#ifndef ORG_EEROS_CONTROL_DEMUX_HPP_
#define ORG_EEROS_CONTROL_DEMUX_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

namespace eeros {
	namespace control {
		
		template < uint32_t N, typename T = double, typename C = eeros::math::Matrix<N,1,T> >
		class DeMux: public Block {
		public:
			DeMux() { 
				for(int i = 0; i < N; i++) this->out[i].getSignal().clear();
			}
			
			virtual void run() {
				for(int i = 0; i < N; i++) {
					out[i].getSignal().setValue(in.getSignal().getValue()(i));
					out[i].getSignal().setTimestamp(in.getSignal().getTimestamp());
				}
			}
			
			virtual Input<C>& getIn() {
				return in;
			}
			
			virtual Output<T>& getOut(uint32_t index) {
				return out[index];
			}
			
		protected:
			Input<C> in;
			Output<T> out[N];
		};

	};
};

#endif /* ORG_EEROS_CONTROL_DEMUX_HPP_ */
