#ifndef ORG_EEROS_CONTROL_CONSTANT_HPP_
#define ORG_EEROS_CONTROL_CONSTANT_HPP_

#include <type_traits>
#include <mutex>
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
			
			Constant(T v) : value(v) { }
			
			virtual void run() {
				std::lock_guard<std::mutex> lock(mtx);
				this->out.getSignal().setValue(value);
				this->out.getSignal().setTimestamp(System::getTimeNs());
			}
			
			virtual void setValue(T newValue) {
				std::lock_guard<std::mutex> lock(mtx);
				value = newValue;
			}

			virtual T getValue () const {
				return value;
			}

			template <typename X>
			friend std::ostream& operator<<(std::ostream& os, Constant<X>& c);

		protected:
			T value;
			std::mutex mtx;
			
		private:
			template <typename S> typename std::enable_if<std::is_integral<S>::value>::type _clear() {
				value = std::numeric_limits<int32_t>::min();
			}
			template <typename S> typename std::enable_if<std::is_floating_point<S>::value>::type _clear() {
				value = std::numeric_limits<double>::quiet_NaN();
			}
			template <typename S> typename std::enable_if<!std::is_arithmetic<S>::value && std::is_integral<typename S::value_type>::value>::type _clear() {
				value.fill(std::numeric_limits<int32_t>::min());
			}
			template <typename S> typename std::enable_if<   !std::is_arithmetic<S>::value && std::is_floating_point<typename S::value_type>::value>::type _clear() {
				value.fill(std::numeric_limits<double>::quiet_NaN());
			}
		};
		
		/********** Print functions **********/
		template <typename T>
		std::ostream& operator<<(std::ostream& os, Constant<T>& c) {
			os << "Block constant: '" << c.getName() << "' init val = " << c.value; 
		}
	};
};

#endif /* ORG_EEROS_CONTROL_CONSTANT_HPP_ */
