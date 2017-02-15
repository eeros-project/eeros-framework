#ifndef ORG_EEROS_CONTROL_D_HPP_
#define ORG_EEROS_CONTROL_D_HPP_

#include <eeros/control/Block1i1o.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class D: public Block1i1o<T> {
			
		public:
			D() : first(true) {
				this->out.getSignal().clear();
			}
			
			virtual void run() {
				if(first) {  // first run, no previous value available -> set output to zero
					prev = this->in.getSignal();
					this->out.getSignal().clear();
					this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
					first = false;
				}
				else {
					double tin = this->in.getSignal().getTimestamp() / 1000000000.0;
					double tprev = this->prev.getTimestamp() / 1000000000.0;
					T valin = this->in.getSignal().getValue();
					T valprev = this->prev.getValue();
					
					this->out.getSignal().setValue((valin - valprev) / (tin - tprev));
					this->out.getSignal().setTimestamp((this->in.getSignal().getTimestamp() + this->prev.getTimestamp()) / 2);
					
					prev = this->in.getSignal();
				}
			}
			
		protected:
			Signal<T> prev;
			bool first;
		};
	};
};
#endif /* ORG_EEROS_CONTROL_D_HPP_ */
