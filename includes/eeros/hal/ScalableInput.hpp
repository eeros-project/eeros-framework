#ifndef ORG_EEROS_HAL_SCALABLEINPUT_HPP_
#define ORG_EEROS_HAL_SCALABLEINPUT_HPP_

#include <string>
#include <eeros/hal/Input.hpp>

namespace eeros {
	namespace hal {

		template <typename T>
		class ScalableInput : public Input<T> {
		public:
			ScalableInput(std::string id, T scale, T offset, std::string unit = "") : Input<T>(id), scale(scale), offset(offset), unit(unit) { }
			virtual ~ScalableInput() { }
			
			virtual T getScale() { return scale; }
			virtual T getOffset() { return offset; }
			virtual std::string getUnit() { return unit; }
			virtual void setScale(T s) { scale = s; }
			virtual void setOffset(T o) { offset = o; }
			virtual void setUnit(std::string unit) { this->unit = unit; }
			
		protected:
			T scale;
			T offset;
			std::string unit;
		};

	};
};

#endif /* ORG_EEROS_HAL_SCALABLEINPUT_HPP_ */
