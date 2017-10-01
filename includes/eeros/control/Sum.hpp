#ifndef ORG_EEROS_CONTROL_SUM_HPP_
#define ORG_EEROS_CONTROL_SUM_HPP_

#include <eeros/control/Block1o.hpp>
#include <eeros/control/Input.hpp>

namespace eeros {
	namespace control {

		template < uint8_t N = 2, typename T = double >
		class Sum : public Block1o<T> {

		public:
			Sum() {
				for(uint8_t i = 0; i < N; i++) {
					negated[i] = false;
				}
			}

			virtual void run() {
				T sum; sum = 0; // TODO works only with primitive types or eeros::math::Matrix -> make specialization and use fill() for compatibility with std::array;
				for(uint8_t i = 0; i < N; i++) {
					if(negated[i]) sum -= in[i].getSignal().getValue();
					else sum += in[i].getSignal().getValue();
				}
				this->out.getSignal().setValue(sum);
				this->out.getSignal().setTimestamp(in[0].getSignal().getTimestamp());
			}
			
			virtual Input<T>& getIn(uint8_t index) {
				return in[index];
			}
			
			virtual void negateInput(uint8_t index) {
				negated[index] = true;
			}

		protected:
			Input<T> in[N];
			bool negated[N];
		};
		
		/********** Print functions **********/
		template <uint8_t N, typename T>
		std::ostream& operator<<(std::ostream& os, Sum<N,T>& sum) {
			os << "Block sum: '" << sum.getName() << "'"; 
		}

	};
};

#endif /* ORG_EEROS_CONTROL_SUM_HPP_ */
