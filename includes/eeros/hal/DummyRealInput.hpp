#ifndef ORG_EEROS_HAL_DUMMYREALINPUT_HPP_
#define ORG_EEROS_HAL_DUMMYREALINPUT_HPP_

#include <string>
#include <eeros/hal/ScalableInput.hpp>

namespace eeros {
	namespace hal {

		class DummyRealInput : public ScalableInput<double> {
		public:
			DummyRealInput(std::string id, void* libHandle = nullptr, double scale = 1, double offset = 0, double minIn = -10.0, double maxIn = -10.0);
			virtual double get();
		};
	};
};

#endif /* ORG_EEROS_HAL_DUMMYREALINPUT_HPP_ */
