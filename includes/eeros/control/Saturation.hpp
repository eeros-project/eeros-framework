#ifndef ORG_EEROS_CONTROL_SATURATION_HPP_
#define ORG_EEROS_CONTROL_SATURATION_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <eeros/math/Matrix.hpp>

namespace eeros {
	namespace control {
		
		template<typename T = double>
		class Saturation : public Block1i1o<T> {
			
		public:
			Saturation() : enabled(false) {
				lowerLimit = 0;
				upperLimit = 0;
			}
			
			Saturation(T sym) : enabled(true) {
				lowerLimit = -sym;
				upperLimit = sym;
			}
			
			Saturation(T lower, T upper) : enabled(true) {
				lowerLimit = lower;
				upperLimit = upper;
			}
			
			virtual void run() {
				T outVal = this->in.getSignal().getValue();
				if(enabled) {
					if(outVal > upperLimit) outVal = upperLimit;
					if(outVal < lowerLimit) outVal = lowerLimit;
				}
				this->out.getSignal().setValue(outVal);
				this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
			}
			
			virtual void enable() {
				enabled = true;
			}
			
			virtual void disable() {
				enabled = false;
			}
			
			virtual void setLimit(T lower, T upper) {
				lowerLimit = lower;
				upperLimit = upper;
			}
			
		protected:
			T lowerLimit, upperLimit;
			bool enabled;
		};
		
	};
};

#endif /* ORG_EEROS_CONTROL_SATURATION_HPP_ */
