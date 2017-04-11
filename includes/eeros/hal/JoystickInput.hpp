#ifndef ORG_EEROS_HAL_JOYSTICKINPUT_HPP_
#define ORG_EEROS_HAL_JOYSTICKINPUT_HPP_

#include <string>
#include <eeros/hal/Input.hpp>
#include <atomic>

namespace eeros {
	namespace hal {

		template <typename T>
		class JoystickInput : public Input<T> {
		public:
			JoystickInput(std::string id, void* libHandle) : Input<T>(id, libHandle) { }
			virtual ~JoystickInput() { }
			
			virtual bool get() { return value.load(); }
			virtual void setValue(bool val) { value.store(val); }
			
		private:
			std::atomic<bool> value;
		};

	};
};

#endif /* ORG_EEROS_HAL_JOYSTICKINPUT_HPP_ */