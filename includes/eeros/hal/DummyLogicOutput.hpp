#ifndef ORG_EEROS_HAL_DUMMYLOGICOUTPUT_HPP_
#define ORG_EEROS_HAL_DUMMYLOGICOUTPUT_HPP_

#include <string>
#include <eeros/hal/Output.hpp>

namespace eeros {
	namespace hal {

		class DummyLogicOutput : public Output<bool> {
		public:
			DummyLogicOutput(std::string id);
			virtual bool get();
			virtual void set(bool value);
		};

	};
};

#endif /* ORG_EEROS_HAL_DUMMYLOGICOUTPUT_HPP_ */
