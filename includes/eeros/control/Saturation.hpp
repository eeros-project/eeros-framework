#ifndef ORG_EEROS_CONTROL_SATURATION_HPP_
#define ORG_EEROS_CONTROL_SATURATION_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <eeros/math/Matrix.hpp>
#include <type_traits>

namespace eeros {
	namespace control {
		
		template<typename T = double, typename enable = void>
		class Saturation;
		
		// Non arithmetic types (container types like std::vector or eeros::math::Matrix)
		template<typename T>
		class Saturation<T, typename std::enable_if<!std::is_arithmetic<T>::value >::type> : public Block1i1o<T> {
			
		public:
			Saturation() : enabled(false) {
				for(unsigned int i = 0; i < lowerLimit.size(); i++) {
					lowerLimit[i] = 0;
					upperLimit[i] = 0;
				}
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
					for(unsigned int i = 0; i < outVal.size(); i++) {
						if(outVal[i] > upperLimit[i]) outVal[i] = upperLimit[i];
						if(outVal[i] < lowerLimit[i]) outVal[i] = lowerLimit[i];
					}
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
		
		// Arithmetic types
		template<typename T>
		class Saturation<T, typename std::enable_if<std::is_arithmetic<T>::value >::type> : public Block1i1o<T> {
			
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
