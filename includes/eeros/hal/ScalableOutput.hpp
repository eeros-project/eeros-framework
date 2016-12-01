#ifndef ORG_EEROS_HAL_SCALABLEOUTPUT_HPP_
#define ORG_EEROS_HAL_SCALABLEOUTPUT_HPP_

#include <eeros/core/System.hpp>
#include <eeros/hal/Output.hpp>

namespace eeros {
	namespace hal {

		template <typename T>
		class ScalableOutput : public Output<T> {
		public:
			explicit ScalableOutput(std::string id, T scale, T offset) : Output<T>(id), scale(scale), offset(offset) { }
			virtual ~ScalableOutput() { }
			
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

#endif /* ORG_EEROS_HAL_SCALABLEOUTPUT_HPP_ */
