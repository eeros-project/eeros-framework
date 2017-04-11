#ifndef ORG_EEROS_HAL_DUMMYREALOUTPUT_HPP_
#define ORG_EEROS_HAL_DUMMYREALOUTPUT_HPP_

#include <string>
#include <eeros/hal/ScalableOutput.hpp>

namespace eeros {
	namespace hal {

		class DummyRealOutput : public ScalableOutput<double> {
		public:
			DummyRealOutput(std::string id, void* libHandle = nullptr, double scale = 1, double offset = 0, double minOut = -10.0, double maxOut = 10.0);
			virtual double get();
			virtual void set(double value);
		};

	};
};

#endif /* ORG_EEROS_HAL_DUMMYREALOUTPUT_HPP_ */
