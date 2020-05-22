#ifndef ORG_EEROS_CONTROL_I_HPP_
#define ORG_EEROS_CONTROL_I_HPP_

#include <eeros/control/Block1i1o.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class I: public Block1i1o<T> {
		public:
			I() : first(true), enabled(false) { prev.clear(); clearLimits(); }
			
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
					if(enabled) {
						T val = valprev + valin * dt;
						if ((val < upperLimit) && (val > lowerLimit)) output = val; 
						else output = valprev;
					} else
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
			virtual void setLimit(T upper, T lower) {
				this->upperLimit = upper;
				this->lowerLimit = lower;
				T val = prev.getValue();
				if (val > upper) prev.setValue(upper);
				if (val < lower) prev.setValue(lower);
			}
			
		protected:
			bool first;
			bool enabled;
			Signal<T> prev;
			T upperLimit, lowerLimit;
		private:
			virtual void clearLimits() {
				_clear<T>();
			}
			template <typename S> typename std::enable_if<std::is_integral<S>::value>::type _clear() {
				upperLimit = std::numeric_limits<int32_t>::max();
				lowerLimit = -upperLimit;
			}
			template <typename S> typename std::enable_if<std::is_floating_point<S>::value>::type _clear() {
				upperLimit = std::numeric_limits<double>::max();
				lowerLimit = -upperLimit;
			}
			template <typename S> typename std::enable_if<!std::is_arithmetic<S>::value && std::is_integral<typename S::value_type>::value>::type _clear() {
				upperLimit.fill(std::numeric_limits<int32_t>::max());
				lowerLimit = -upperLimit;
			}
			template <typename S> typename std::enable_if<   !std::is_arithmetic<S>::value && std::is_floating_point<typename S::value_type>::value>::type _clear() {
				upperLimit.fill(std::numeric_limits<double>::max());
				lowerLimit = -upperLimit;
			}

		};
		
		/********** Print functions **********/
		template <typename T>
		std::ostream& operator<<(std::ostream& os, I<T>& i) {
			os << "Block integrator: '" << i.getName(); 
            return os;
		}
	};
};
#endif /* ORG_EEROS_CONTROL_I_HPP_ */
