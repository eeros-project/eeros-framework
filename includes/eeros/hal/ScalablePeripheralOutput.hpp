#ifndef ORG_EEROS_HAL_SCALABLEPERIPHERALOUTPUT_HPP_
#define ORG_EEROS_HAL_SCALABLEPERIPHERALOUTPUT_HPP_

#include <eeros/core/System.hpp>
#include <eeros/hal/PeripheralOutput.hpp>

namespace eeros {
	namespace hal {

		template <typename T>
		class ScalablePeripheralOutput : public PeripheralOutput<T> {
		public:
			explicit ScalablePeripheralOutput(std::string id, T scale, T offset) : PeripheralOutput<T>(id), scale(scale), offset(offset) { }
			virtual ~ScalablePeripheralOutput() { }
			
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

#endif /* ORG_EEROS_HAL_SCALABLEPERIPHERALOUTPUT_HPP_ */
