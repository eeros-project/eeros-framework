#ifndef ORG_EEROS_HAL_JOYSTICK_HPP_
#define ORG_EEROS_HAL_JOYSTICK_HPP_

#include <string>
#include <functional>
#include <linux/joystick.h>

#define JOYSTICK_BUTTON_COUNT (16)
#define JOYSTICK_AXIS_COUNT (8)

namespace eeros {
	namespace hal {
		struct JoystickState
		{
			double axis[JOYSTICK_AXIS_COUNT];
			bool button_state[JOYSTICK_BUTTON_COUNT];
			bool button_up[JOYSTICK_BUTTON_COUNT];
			bool button_down[JOYSTICK_BUTTON_COUNT];
			
			static const double axis_max;
		};
		
		class Joystick
		{
		public:
			explicit Joystick();
			~Joystick();
			virtual bool open(const char* device);
			virtual void close();
			virtual void loop();
			virtual void on_event(std::function<void(struct js_event)> action);
			virtual void on_button(std::function<void(int, bool)> action);
			virtual void on_axis(std::function<void(int, double)> action);
			
			virtual std::string name();
			
			JoystickState last;
			JoystickState current;
			
		private:
			int fd;
			std::function<void(struct js_event)> event_action;
			std::function<void(int, bool)> button_action;
			std::function<void(int, double)> axis_action;
		};
	}
}

#endif // ORG_EEROS_HAL_JOYSTICK_HPP_
