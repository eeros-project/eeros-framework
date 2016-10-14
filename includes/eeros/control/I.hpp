#ifndef ORG_EEROS_CONTROL_I_HPP_
#define ORG_EEROS_CONTROL_I_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/core/System.hpp>

#include <iostream>

namespace eeros {
	namespace control {

		template < typename T = double >
		class I: public eeros::control::Block {
			
		public:
			I() : first(true) { }
			
			
			virtual void run() {
				if(first) {  // first run, no previous value available -> set output to zero
					this->out.getSignal().clear();
					this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
					this->prev = this->out.getSignal();
					first = false;
				}
				else {
					double tin = this->in.getSignal().getTimestamp() / 1000000000.0;
					double tprev = this->prev.getTimestamp() / 1000000000.0;
					
					double dt = (tin - tprev);
					T valin = this->in.getSignal().getValue();
					T valprev = this->prev.getValue();
					
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
			virtual void setInitCondition(T pos) {
				this->prev.setValue(pos);
				this->prev.setTimestamp(this->out.getSignal().getTimestamp());
			}
			
			virtual eeros::control::Input<T>& getIn() {
				return in;
			}
			virtual eeros::control::Output<T>& getOut() {
				return out;
			}
			virtual eeros::control::Input<bool>& getEnable() {
				return enable_ext;
			}

			eeros::control::Signal<T> prev;
			
// 		protected:
			bool first;
			bool enabled = false;
			T output; 
			
			eeros::control::Input<T> in;
			eeros::control::Output<T> out;
			eeros::control::Input<bool> enable_ext;
			
		};
	};
};
#endif /* ORG_EEROS_CONTROL_I_HPP_ */
