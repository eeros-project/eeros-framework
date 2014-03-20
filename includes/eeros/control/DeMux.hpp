#ifndef ORG_EEROS_CONTROL_DEMUX_HPP_
#define ORG_EEROS_CONTROL_DEMUX_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/types.hpp>
#include <vector>

namespace eeros {
	namespace control {
		
		template < uint32_t N, typename T = double >
		class DeMux: public Block {
		public:
			DeMux() { }
			
			virtual void run() {
				timestamp_t t = in.getSignal().getTimestamp();
				for(int i = 0; i < N; i++) {
					out[i].setValue(in.getSignal().getValue()(i));
					out[i].setTimeStamp(t);
				}
			}
			
			virtual Input<T>& getIn() {
				return in;
			}
			
			virtual Output<T>& getOut(uint32_t output) {
				return out[output];
			}
			
		protected:
			Input<T> in;
			Output<T> out[N];
		};

	};
};

#endif /* ORG_EEROS_CONTROL_DEMUX_HPP_ */
