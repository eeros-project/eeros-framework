#ifndef ORG_EEROS_HAL_DUMMYLOGICOUTPUT_HPP_
#define ORG_EEROS_HAL_DUMMYLOGICOUTPUT_HPP_

#include <string>
#include <eeros/hal/PeripheralOutput.hpp>

namespace eeros {
	namespace hal {

		class DummyLogicOutput : public PeripheralOutput<bool> {
		public:
			DummyLogicOutput(std::string id);
			virtual bool get();
			virtual void set(bool value);
		private:
			bool value;
		};

	};
};

#endif /* ORG_EEROS_HAL_DUMMYLOGICOUTPUT_HPP_ */
