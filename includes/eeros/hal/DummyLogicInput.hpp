#ifndef ORG_EEROS_HAL_DUMMYLOGICINPUT_HPP_
#define ORG_EEROS_HAL_DUMMYLOGICINPUT_HPP_

#include <string>
#include <eeros/hal/Input.hpp>

namespace eeros {
	namespace hal {

		class DummyLogicInput : public Input<bool> {
		public:
			DummyLogicInput(std::string id);
			virtual bool get();
		};

	};
};

#endif /* ORG_EEROS_HAL_DUMMYLOGICINPUT_HPP_ */
