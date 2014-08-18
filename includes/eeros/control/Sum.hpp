#ifndef ORG_EEROS_CONTROL_SUM_HPP_
#define ORG_EEROS_CONTROL_SUM_HPP_

#include <vector>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

namespace eeros {
	namespace control {

		template < uint8_t N = 2, typename T = double >
		class Sum : public Block {

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
				out.getSignal().setValue(sum);
				out.getSignal().setTimestamp(in[0].getSignal().getTimestamp());
			}
			
			virtual Input<T>& getIn(uint8_t index) {
				return in[index];
			}
			
			virtual Output<T>& getOut() {
				return out;
			}
			
			
			virtual void negateInput(uint8_t index) {
				negated[index] = true;
			}

		protected:
			Input<T> in[N];
			Output<T> out;
			bool negated[N];
		};

	};
};

#endif /* ORG_EEROS_CONTROL_SUM_HPP_ */
