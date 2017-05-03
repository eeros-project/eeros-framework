#ifndef ORG_EEROS_HAL_XBOX_HPP_
#define ORG_EEROS_HAL_XBOX_HPP_

#include <string>
#include <functional>
#include <linux/joystick.h>
#include <eeros/hal/Input.hpp>

#define XBOX_BUTTON_COUNT (8)
#define XBOX_AXIS_COUNT (8)

namespace eeros {
	namespace hal {
		struct XBoxState {
			double axis[XBOX_AXIS_COUNT];
			bool button_state[XBOX_BUTTON_COUNT];
			bool button_up[XBOX_BUTTON_COUNT];
			bool button_down[XBOX_BUTTON_COUNT];
			
			static const double axis_max;
		};
		
		struct XBoxController {
			struct Axis {
				static constexpr int LX = 0;
				static constexpr int LY = 1;
				static constexpr int LT = 2;
				static constexpr int RY = 3;
				static constexpr int RX = 4;
				static constexpr int RT = 5;
				static constexpr int CX = 6;
				static constexpr int CY = 7;
			};
			struct Button {
				static constexpr int A = 0;
				static constexpr int B = 1;
				static constexpr int X = 2;
				static constexpr int Y = 3;
				static constexpr int LB = 4;
				static constexpr int RB = 5;
				static constexpr int back = 6;
				static constexpr int start = 7;
			};
		};
		
		class XBox {
		public:
			explicit XBox();
			~XBox();
			virtual bool open(const char* device);
			virtual void close();
			virtual void loop();
			virtual void on_event(std::function<void(struct js_event)> action);
			virtual void on_button(std::function<void(int, bool)> action);
			virtual void on_axis(std::function<void(int, double)> action);
			
			virtual std::string name();
			
			XBoxState last;
			XBoxState current;
			
		private:
			int fd;
			std::function<void(struct js_event)> event_action;
			std::function<void(int, bool)> button_action;
			std::function<void(int, double)> axis_action;
			Input<bool>* button[XBOX_BUTTON_COUNT];
		};
	}
}

#endif // ORG_EEROS_HAL_XBOX_HPP_
