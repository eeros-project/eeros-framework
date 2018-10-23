#include <eeros/hal/XBox.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/XBoxDigIn.hpp>

#include <cstdio>
#include <fcntl.h>
#include <unistd.h>

using namespace eeros::hal;

const double XBoxState::axis_max = 0x7fff;

XBox::XBox(std::string dev) {
	open(dev.c_str());
	button[0] = new XBoxDigIn("XBoxButtonA", this);
	button[1] = new XBoxDigIn("XBoxButtonB", this);
	button[2] = new XBoxDigIn("XBoxButtonX", this);
	button[3] = new XBoxDigIn("XBoxButtonY", this);
	button[4] = new XBoxDigIn("XBoxButtonLB", this);
	button[5] = new XBoxDigIn("XBoxButtonRB", this);
	button[6] = new XBoxDigIn("XBoxButtonBack", this);
	button[7] = new XBoxDigIn("XBoxButtonStart", this);
	HAL& hal = HAL::instance();
	for (int i = 0; i < XBOX_BUTTON_COUNT; i++) hal.addInput(button[i]);

	for (int i = 0; i < XBOX_AXIS_COUNT; i++) 	{
		last.axis[i] = 0;
		current.axis[i] = 0;
	}
	for (int i = 0; i < XBOX_BUTTON_COUNT; i++)	{
		last.button_state[i] = false;
		last.button_up[i] = false;
		last.button_down[i] = false;
		current.button_state[i] = false;
		current.button_up[i] = false;
		current.button_down[i] = false;
	}
}


XBox::~XBox() {
	running = false; 
	join(); 
	close(); 
}

bool XBox::open(const char* device) {
	fd = ::open(device, O_RDONLY);
	if (fd < 0) log.error() << "XBox: could not open input device on " + std::string(device);
	return fd;
}

void XBox::close() { ::close(fd); }

std::string XBox::name() {
	if (!fd) return "";
	
	char name[128];
	if (ioctl(fd, JSIOCGNAME (sizeof(name)), name)) {
		name[127] = 0;
		return name;
	} else return "";
}

void XBox::on_event(std::function<void(struct js_event)> action) {
	event_action = action;
}

void XBox::on_button(std::function<void(int, bool)> action) {
	button_action = action;
}

void XBox::on_axis(std::function<void(int, double)> action) {
	axis_action = action;
}


void XBox::run() {
	running = true;
	if (fd < 0) return;
	struct js_event e;
	while (running) 	{
		ssize_t n = read(fd, &e, sizeof(struct js_event));
		
		switch (e.type) {
			case (JS_EVENT_BUTTON | JS_EVENT_INIT):
			case JS_EVENT_BUTTON:
				if (e.number < XBOX_BUTTON_COUNT) {
					current.button_state[e.number] = e.value;
					current.button_up[e.number] = (!current.button_state[e.number] & last.button_state[e.number]);
					current.button_down[e.number] = (current.button_state[e.number] & !last.button_state[e.number]);
					
					if (e.type != (JS_EVENT_BUTTON | JS_EVENT_INIT))
						if (button_action != nullptr)
							button_action(e.number, e.value);
				}
				break;
				
			case (JS_EVENT_AXIS | JS_EVENT_INIT):
			case JS_EVENT_AXIS:
				if (e.number < XBOX_AXIS_COUNT) {
					current.axis[e.number] = (e.value / XBoxState::axis_max);
					
					if (e.type != (JS_EVENT_AXIS | JS_EVENT_INIT))
						if (axis_action != nullptr)
							axis_action(e.number, current.axis[e.number]);
				}
				break;
				
			default:
				break;
		}
		
		if (event_action != nullptr) event_action(e);

		last = current;
	}
}
