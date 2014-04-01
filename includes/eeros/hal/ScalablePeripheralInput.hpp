#ifndef ORG_EEROS_HAL_SCALABLEPERIPHERALINPUT_HPP_
#define ORG_EEROS_HAL_SCALABLEPERIPHERALINPUT_HPP_

#include <string>
#include <eeros/hal/PeripheralInput.hpp>

namespace eeros {
	namespace hal {

		template <typename T>
		class ScalablePeripheralInput : public PeripheralInput<T> {
		public:
			ScalablePeripheralInput(std::string id, T scale, T offset) : PeripheralInput<T>(id), scale(scale), offset(offset) { }
			virtual ~ScalablePeripheralInput() { }
			
			virtual T getScale() { return scale; }
			virtual T getOffset() { return offset; }
			virtual void setScale(T s) { scale = s; }
			virtual void setOffset(T o) { offset = o; }
			
		protected:
			T scale;
			T offset;
		};

	};
};

#endif /* ORG_EEROS_HAL_SCALABLEPERIPHERALINPUT_HPP_ */
