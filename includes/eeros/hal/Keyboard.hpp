#ifndef ORG_EEROS_HAL_KEYBOARD_HPP_
#define ORG_EEROS_HAL_KEYBOARD_HPP_

#include <string>
#include <functional>
#include <termios.h>
#include <linux/input.h>
#include <eeros/hal/Input.hpp>
#include <eeros/core/Thread.hpp>

namespace eeros {
	namespace hal {
		
		struct Events {
			bool esc;
			bool emergency;
			bool reset;
			bool start;
			bool stop;
		};

		class Keyboard : public eeros::Thread {
		public:
			explicit Keyboard(int priority);
			~Keyboard();
			Events events;
			bool homed[5];
			double speed[4];
		private:
			virtual void run();
			bool running;
			struct termios tio;
			Input<bool>* esc;
			Input<bool>* emergency;
			Input<bool>* reset;
			Input<bool>* start;
			Input<bool>* stop;
		};
	}
}

#endif // ORG_EEROS_HAL_KEYBOARD_HPP_
