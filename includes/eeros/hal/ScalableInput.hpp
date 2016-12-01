#ifndef ORG_EEROS_HAL_SCALABLEINPUT_HPP_
#define ORG_EEROS_HAL_SCALABLEINPUT_HPP_

#include <string>
#include <eeros/hal/Input.hpp>

namespace eeros {
	namespace hal {

		template <typename T>
		class ScalableInput : public Input<T> {
		public:
			ScalableInput(std::string id, T scale, T offset) : Input<T>(id), scale(scale), offset(offset) { }
			virtual ~ScalableInput() { }
			
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

#endif /* ORG_EEROS_HAL_SCALABLEINPUT_HPP_ */
