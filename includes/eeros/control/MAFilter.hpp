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
			

			template <size_t No, typename ValT, typename CoeffT>
			friend std::ostream& operator<<(std::ostream& os, MAFilter<No,ValT,CoeffT>& filter);

		protected:
			Tcoeff * coefficients;
			Tval previousValues[N]{};
			bool enabled{true};
		};
		
		
		/********** Print functions **********/
		template <size_t N, typename Tval, typename Tcoeff>
		std::ostream& operator<<(std::ostream& os, MAFilter<N,Tval,Tcoeff>& filter) {
			os << "Block MAFilter: '" << filter.getName() << "' is enabled=" << filter.enabled << ", coefficients:[";

			os << filter.coefficients[0];
			for(size_t i = 1; i < N; i++){
				os << "," << filter.coefficients[i];
			}
			os << "], previousValues:[";

			os << filter.previousValues[0];
			for(size_t i = 1; i < N; i++){
				os << "," << filter.previousValues[i];
			}
			os << "]";
		}
	};
};

#endif /* ORG_EEROS_CONTROL_MAFILTER_HPP_ */
