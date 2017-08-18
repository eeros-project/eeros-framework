#ifndef ORG_EEROS_CONTROL_I_HPP_
#define ORG_EEROS_CONTROL_I_HPP_

#include <eeros/control/Block1i1o.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class I: public Block1i1o<T> {
		public:
			I() : first(true), enabled(false) { prev.clear(); }
			
			virtual void run() {
				if(first) {  // first run, set output to init condition
 					this->out.getSignal().setValue(this->prev.getValue());
					this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
					this->prev.setTimestamp(this->out.getSignal().getTimestamp());
					first = false;
				}
				else {
					double tin = this->in.getSignal().getTimestamp() / 1000000000.0;
					double tprev = this->prev.getTimestamp() / 1000000000.0;
					
					double dt = (tin - tprev);
					T valin = this->in.getSignal().getValue();
					T valprev = this->prev.getValue();
					T output;
					if(enabled)
						output = valprev + valin * dt;
					else
						output = valprev;

					this->out.getSignal().setValue(output);
					this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
					this->prev = this->out.getSignal();
				}
			}

			virtual void enable() {
				this->enabled = true;
			}
			virtual void disable() {
				this->enabled = false;
			}
			virtual void setInitCondition(T val) {
				this->prev.setValue(val);
			}
			
		protected:
			bool first;
			bool enabled;
			Signal<T> prev;
		};
		
		/********** Print functions **********/
		template <typename T>
		std::ostream& operator<<(std::ostream& os, I<T>& i) {
			os << "Block integrator: '" << i.getName(); 
		}
	};
};
#endif /* ORG_EEROS_CONTROL_I_HPP_ */
