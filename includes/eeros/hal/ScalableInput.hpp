#ifndef ORG_EEROS_HAL_SCALABLEINPUT_HPP_
#define ORG_EEROS_HAL_SCALABLEINPUT_HPP_

#include <string>
#include <eeros/hal/Input.hpp>

namespace eeros {
	namespace hal {

		template <typename T>
		class ScalableInput : public Input<T> {
		public:
			ScalableInput(std::string id, void* libHandle, T scale, T offset, T minIn, T maxIn, std::string unit = "") : Input<T>(id, libHandle), scale(scale), offset(offset), minIn(minIn), maxIn(maxIn), unit(unit)  { }
			virtual ~ScalableInput() { }
			
			virtual T getScale() { return scale; }
			virtual T getOffset() { return offset; }
			virtual std::string getUnit() { return unit; }
			virtual T getMinIn() { return minIn; }
			virtual T getMaxIn() { return maxIn; }
			virtual void setScale(T s) { scale = s; }
			virtual void setOffset(T o) { offset = o; }
			virtual void setUnit(std::string unit) { this->unit = unit; }
			virtual void setMinIn(T minI) { minIn = minI; }
			virtual void setMaxIn(T maxI) { maxIn = maxI; }
			
		protected:
			T scale;
			T offset;
			std::string unit;
			T minIn;
			T maxIn;
		};

	};
};

#endif /* ORG_EEROS_HAL_SCALABLEINPUT_HPP_ */
