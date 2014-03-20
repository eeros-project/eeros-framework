#ifndef ORG_EEROS_CONTROL_CONSTANT_HPP_
#define ORG_EEROS_CONTROL_CONSTANT_HPP_

#include <vector>
#include <eeros/control/Block1o.hpp>
#include <eeros/core/System.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class Constant : public Block1o<T> {
		public:
			Constant(T v) {
				value = v;
			}
			
			virtual void run() {
				this->out.getSignal().setValue(value);
				this->out.getSignal().setTimestamp(System::getTimeNs());
			}
			
			virtual void setValue(T newValue) {
				value = newValue;
			}
			
		protected:
			T value;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_CONSTANT_HPP_ */
