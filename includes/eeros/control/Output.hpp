#ifndef ORG_EEROS_CONTROL_OUTPUT_HPP_
#define ORG_EEROS_CONTROL_OUTPUT_HPP_

#include <eeros/control/Signal.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class Output {
		
		public:
			virtual Signal<T>& getSignal() {
				return signal;
			}
			
		private:
			Signal<T> signal;
		};
	};
};

#endif /* ORG_EEROS_CONTROL_OUTPUT_HPP_ */
