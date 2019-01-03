#ifndef ORG_EEROS_CONTROL_MAFILTER_HPP_
#define ORG_EEROS_CONTROL_MAFILTER_HPP_

#include <eeros/control/Block1i1o.hpp>

#include <iostream>

namespace eeros {
	namespace control {

		template <size_t N, typename Tval = double, typename Tcoeff = Tval>
		class MAFilter : public Block1i1o<Tval> {
			
		public:
			explicit MAFilter(Tcoeff (& coeff)[N]) : coefficients{coeff} {}
			
			virtual void run() {
				Tval result{};
				for(size_t i = 0; i < N; i++){
					if(i < N-1) {
						previousValues[i] = previousValues[i+1];
					} else {
						previousValues[i] = this->in.getSignal().getValue();
					}
					//std::cout << "i: " << i << " ";
					//std::cout << "resP: " << result << " ";
					//std::cout << "add: " << coefficients[i] * previousValues[i] << " ";
					result += coefficients[i] * previousValues[i];
					//std::cout << "resA: " << result << std::endl;
				}
				
				if(enabled) {
					this->out.getSignal().setValue(result);
				  
				} else {
					this->out.getSignal().setValue(this->in.getSignal().getValue());
				}
				
				this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
			}
			
			
			virtual void enable() {
				enabled = true;
			}
			
			virtual void disable() {
				enabled = false;
			}
			

// 			template <typename Xout, typename Xgain>
// 			friend std::ostream& operator<<(std::ostream& os, Gain<Xout,Xgain>& gain);

		protected:
			Tcoeff * coefficients;
			Tval previousValues[N]{};
			bool enabled{true};
		};
		
		
				/********** Print functions **********/
		/*
		template <typename Tout, typename Tgain>
		std::ostream& operator<<(std::ostream& os, Gain<Tout,Tgain>& gain) {
			os << "Block gain: '" << gain.getName() << "' gain val = " << gain.gain; 
		} */

	};
};

#endif /* ORG_EEROS_CONTROL_MAFILTER_HPP_ */
