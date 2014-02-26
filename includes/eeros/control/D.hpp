#ifndef ORG_EEROS_CONTROL_D_HPP_
#define ORG_EEROS_CONTROL_D_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <type_traits>

namespace eeros {
	namespace control {

		template < typename T = double >
		class D: public Block1i1o<T> {
			
		public:
			D() { 
				prev.clear();
			}
			
			virtual void run() {
				this->out.getSignal().setValue((this->in.getSignal().getValue() - prev.getValue()) / ((this->in.getSignal().getTimestamp() - prev.getTimestamp()) / 1000000000.0));
				this->out.getSignal().setTimestamp((this->in.getSignal().getTimestamp() - prev.getTimestamp()) / 2);
				prev = this->out.getSignal();
			}
			
		protected:
			Signal<T> prev;
		};

	};
};
#endif /* ORG_EEROS_CONTROL_D_HPP_ */
