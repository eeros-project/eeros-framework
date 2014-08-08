#ifndef ORG_EEROS_CONTROL_CONSTANT_HPP_
#define ORG_EEROS_CONTROL_CONSTANT_HPP_

#include <type_traits>
#include <eeros/control/Block1o.hpp>
#include <eeros/core/System.hpp>

namespace eeros {
	namespace control {

		template < typename T = double >
		class Constant : public Block1o<T> {
		public:
			Constant() {
				_clear<T>();
			}
			
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
			
		private:
			template <typename S> typename std::enable_if<std::is_arithmetic<S>::value>::type _clear() {
				value = 0;
			}
			
			template <typename S> typename std::enable_if<!std::is_arithmetic<S>::value>::type _clear() {
				value.fill(0);
			}
		};
	};
};

#endif /* ORG_EEROS_CONTROL_CONSTANT_HPP_ */
