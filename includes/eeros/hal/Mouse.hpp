#ifndef ORG_EEROS_HAL_MOUSE_HPP_
#define ORG_EEROS_HAL_MOUSE_HPP_

#include <string>
#include <functional>
#include <linux/input.h>

#define MOUSE_BUTTON_COUNT (16)
#define MOUSE_AXIS_COUNT (8)

namespace eeros {
	namespace hal {

        struct MouseState
        {
                struct
                {
                        bool left;
                        bool middle;
                        bool right;
                } button;
                struct
                {
                        signed x;
                        signed y;
                        signed z;
                        signed r;
                } axis;
        };

        class Mouse
        {
        public:
                explicit Mouse();
                ~Mouse();
                virtual bool open(const char* device);
                virtual void close();
                virtual void loop();
                virtual void on_event(std::function<void(struct input_event)> action);
                virtual void on_button(std::function<void(int, bool)> action);
                virtual void on_axis(std::function<void(int, signed)> action);

                virtual std::string name();

                MouseState current;
                MouseState last;

        private:
                int fd;
                std::function<void(struct input_event)> event_action;
                std::function<void(int, bool)> button_action;
                std::function<void(int, signed)> axis_action;
        };
	}
}

#endif // ORG_EEROS_HAL_MOUSE_HPP_
