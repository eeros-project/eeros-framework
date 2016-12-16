#ifndef ORG_EEROS_HAL_SCALABLEOUTPUT_HPP_
#define ORG_EEROS_HAL_SCALABLEOUTPUT_HPP_

#include <eeros/core/System.hpp>
#include <eeros/hal/Output.hpp>

namespace eeros {
	namespace hal {

		template <typename T>
		class ScalableOutput : public Output<T> {
		public:
			explicit ScalableOutput(std::string id, void* libHandle, T scale, T offset, T minOut, T maxOut, std::string unit = "" ) : Output<T>(id, libHandle), scale(scale), offset(offset), minOut(minOut), maxOut(maxOut), unit(unit) { }
			virtual ~ScalableOutput() { }
			
			virtual T getScale() { return scale; }
			virtual T getOffset() { return offset; }
			virtual std::string getUnit() { return unit; }
			virtual T getMinOut() { return minOut; }
			virtual T getMaxOut() { return maxOut; }
			virtual void setScale(T s) { scale = s; }
			virtual void setOffset(T o) { offset = o; }
			virtual void setUnit(std::string unit) { this->unit = unit; }
			virtual void setMinOut(T minO) { minOut = minO; }
			virtual void setMaxOut(T maxO) { maxOut = maxO; }
			
		protected:
			T scale;
			T offset;
			std::string unit;
			T minOut;
			T maxOut;
		};

	};
};

#endif /* ORG_EEROS_HAL_SCALABLEOUTPUT_HPP_ */
