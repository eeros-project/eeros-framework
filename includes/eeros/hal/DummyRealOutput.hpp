#ifndef ORG_EEROS_HAL_DUMMYREALOUTPUT_HPP_
#define ORG_EEROS_HAL_DUMMYREALOUTPUT_HPP_

#include <string>
#include <eeros/hal/ScalablePeripheralOutput.hpp>

namespace eeros {
	namespace hal {

		class DummyRealOutput : public ScalablePeripheralOutput<double> {
		public:
			DummyRealOutput(std::string id, double scale = 1, double offset = 0);
			virtual double get();
			virtual void set(double value);
		};

	};
};

#endif /* ORG_EEROS_HAL_DUMMYREALOUTPUT_HPP_ */
