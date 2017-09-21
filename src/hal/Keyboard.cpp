#include <eeros/hal/Keyboard.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/KeyboardDigIn.hpp>

#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>

using namespace eeros::hal;

Keyboard::Keyboard() {
	tcgetattr(STDIN_FILENO, &tio);
	tio.c_lflag &=(~ICANON & ~ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &tio);
	
	esc = new KeyboardDigIn("escKeyboardButton", this);
	emergency = new KeyboardDigIn("emergencyKeyboardButton", this);
	reset = new KeyboardDigIn("resetKeyboardButton", this);
	start = new KeyboardDigIn("startKeyboardButton", this);
	stop = new KeyboardDigIn("stopKeyboardButton", this);
	HAL& hal = HAL::instance();
	hal.addInput(esc);
	hal.addInput(emergency);
	hal.addInput(reset);
	hal.addInput(start);
	hal.addInput(stop);

	events = {false,false,false,false,false};
	for (int i = 0; i < 5; i++) homed[i] = false;
	for (int i = 0; i < 4; i++) speed[i] = 0;
}

Keyboard::~Keyboard() {     
	running = false; 
	join(); 
	tio.c_lflag |=(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &tio);
}

void Keyboard::run() {
	running = true;
	while (running && std::cin.good()) {
		char c;
		std::cin.get(c);

		if (c == 27) events = {true,false,false,false,false};		// ESC
		else if (c == 9) events = {false,true,false,false,false};	// emergency (tab)
		else if (c == 32) events = {false,false,true,false,false};	// reset emergency (shift)
		else if (c == 10) events = {false,false,false,true,false};	// start (enter)
		else if (c == 127) events = {false,false,false,false,true};	// stop (backspace)
		else if (c == '5') for (int i = 0; i < 4; i++) speed[i] = 0;	// speed = 0
		else if (c == '8') {						// forward
			if (speed[0] > -1.0) speed[0] -= 0.02;
                }
		else if (c == '2') {						// backward
			if (speed[0] < 1.0) speed[0] += 0.02;
                }
		else if (c == '4') {						// to the right
			if (speed[1] < 1.0) speed[1] += 0.02;
                }
		else if (c == '6') {						// to the left
			if (speed[1] > -1.0) speed[1] -= 0.02;
                }
		else if (c == '9') {						// turn right
			if (speed[3] > -1.0) speed[3] -= 0.02;
                }
		else if (c == '7') {						// turn left
			if (speed[3] < 1.0) speed[3] += 0.02;
                }
		else if (c == 'q') 
			if (homed[0] == true) homed[0] = false; else homed[0] = true;
		else if (c == 'w') 
			if (homed[1] == true) homed[1] = false; else homed[1] = true;
		else if (c == 'e') 
			if (homed[2] == true) homed[2] = false; else homed[2] = true;
		else if (c == 'r') 
			if (homed[3] == true) homed[3] = false; else homed[3] = true;
		else if (c == 't') 
			if (homed[4] == true) homed[4] = false; else homed[4] = true;
		
		for (int i = 0; i < 4; i++) {
			if (speed[i] < -1.0) speed[i] = -1.0;
			else if (speed[i] > 1.0) speed[i] = 1.0;
		}
        }
}
