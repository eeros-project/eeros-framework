#include <eeros/hal/XBox.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/SpaceNavigatorDigIn.hpp>

#include <cstdio>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>

using namespace eeros::hal;

SpaceNavigator::SpaceNavigator(std::string dev) {
		this->open(dev.c_str());
		this->useRaw = (dev.find("raw") != std::string::npos);
		button[0] = new SpaceNavigatorDigIn("SpaceNavButtonL", this);
		button[1] = new SpaceNavigatorDigIn("SpaceNavButtonR", this);
		HAL& hal = HAL::instance();
		for (int i = 0; i < SPACENAVIGATOR_BUTTON_COUNT; i++) hal.addInput(button[i]);
		for (int i = 0; i < SPACENAVIGATOR_AXIS_COUNT; i++) current.axis[i] = 0;
		for (int i = 0; i < SPACENAVIGATOR_BUTTON_COUNT; i++) current.button[i] = false;
}


SpaceNavigator::~SpaceNavigator() { 
	running = false; 
	join(); 
	this->close(); 
}

bool SpaceNavigator::open(const char* device) {
	file = ::fopen(device, "rb");
	if (file == NULL) {
		log.error() << "Space Navigator: could not open input device on " + std::string(device);
	}
	return true;
}

void SpaceNavigator::close() { ::fclose(file); }

std::string SpaceNavigator::name() {
	if (!file) return "";
	
	char name[128];
	if (ioctl(fileno(file), JSIOCGNAME (sizeof(name)), name)) {
		name[127] = 0;
		return name;
	} else return "";
}

     /* Randy Westlund
     * Space Navigator Project
     *                   PACKET PROTOCOL
     * The space navigator sends packets over the HID stream.  If the stick is not
     * at rest, position packets are sent at 60Hz.  The packet is 14 bytes:
     *
     *    byte 00: always 0x01 -- signals position info to follow
     *    byte 01: xx_low -- the low byte of x position
     *    byte 02: xx_high -- the high byte of x position
     *    byte 03: yy_low
     *    byte 04: y_high
     *    byte 05: zz_low
     *    byte 06: zz_high
     *    byte 07: always 0x02 -- signals rotation info to follow
     *    byte 08: rx_low -- the low byte of x rotation
     *    byte 09: rx_high -- the high byte of x rotation
     *    byte 10: ry_low
     *    byte 11: ry_high
     *    byte 12: rz_low
     *    byte 13: rz_high
     *
     * Values range from -350 to 350.  All values are 0 when the stick is not being
     * touched (no packets are sent in this state).  When the stick is moved in a
     * positive direction, values increase from 0.  When the stick is moved in a
     * negative direction, values decrease from 0xFFFF.
     *
     * Positive x = to your right.  Positive x rotation = toward yourself
     * Positive y = toward yourself.  Positive y rotation = to your left
     * Positive z = down. Positive z rotation = clockwise
     *
     * When a butten press event (depress or release) occurs, a button packet is
     * sent after the current packet finished transmittting.  It is 3 bytes:
     *
     *    byte 0: always 0x03 -- signals button state to follow
     *    byte 1: (left button = 0x01) | (right button = 0x02)
     *    byte 2: always 0x00
     *
     * As such, byte 1 can take values 0x00, 0x01, 0x02, or 0x03.
     */

void SpaceNavigator::run() {
	running = true;
	log.info() << "use raw: " << useRaw;
	if (!file) return;
	if (useRaw) {	// read from raw hid stream
		uint8_t readbuff[14];
		while (running) {
			*readbuff = fgetc(file);
			ssize_t n;
			switch(*readbuff) {
			case 0x01: // position/rotation packet 
				n = fread(readbuff+1, 1, 13, file);
				current.axis[0] = (int16_t)(((int16_t)readbuff[2]<<8)&0xff00) | ((int16_t)readbuff[1]&0xff);
				current.axis[1] = (int16_t)(((int16_t)readbuff[4]<<8)&0xff00) | ((int16_t)readbuff[3]&0xff);
				current.axis[2] = (int16_t)(((int16_t)readbuff[6]<<8)&0xff00) | ((int16_t)readbuff[5]&0xff);
				current.rotAxis[0] = (int16_t)(((int16_t)readbuff[9]<<8)&0xff00) | ((int16_t)readbuff[8]&0xff);
				current.rotAxis[1] = (int16_t)(((int16_t)readbuff[11]<<8)&0xff00) | ((int16_t)readbuff[10]&0xff);
				current.rotAxis[2] = (int16_t)(((int16_t)readbuff[13]<<8)&0xff00) | ((int16_t)readbuff[12]&0xff);
				break;
			case 0x03: // button event
				n = fread(readbuff+1, 1, 2, file); 
				switch(readbuff[1]) {
				case 0x00:
					current.button[0] = false;
					current.button[1] = false;
					break;
				case 0x01:
					current.button[0] = true;
					current.button[1] = false;
					break;
				case 0x02:
					current.button[0] = false;
					current.button[1] = true;
					break;
				case 0x03:
					current.button[0] = true;
					current.button[1] = true;
					break;
				default: 
					break;
				}
				break;
			default: // bad header
				break;
			}
		}
	} else {	// read events
		struct input_event ev;
		while (running) {
			int rd = fread(&ev, sizeof(struct input_event), 1, file);
			switch(ev.type) {
			case 1: // button event
				switch(ev.code) {
				case 256:	// button 0
					if (ev.value) current.button[0] = true;
					else current.button[0] = false;
					break;
				case 257:	// button 1
					if (ev.value) current.button[1] = true;
					else current.button[1] = false;
					break;
				default: 
					break;
				}
				break;
			case 2: // position/rotation packet (rel), sent by: 3Dconnexion SpaceNavigator for Notebooks, vendor 0x46d product 0xc628 version 0x111
				switch(ev.code) {
				case 0:	// X
					current.axis[0] = ev.value;
					break;
				case 1:	// Y
					current.axis[1] = ev.value;
					break;
				case 2:	// Z
					current.axis[2] = ev.value;
					break;
				case 3:	// RY
					current.rotAxis[0] = ev.value;
					break;
				case 4:	// RY
					current.rotAxis[1] = ev.value;
					break;
				case 5:	// RZ
					current.rotAxis[2] = ev.value;
					break;
				default: 
					break;
				}
				break;
			case 3: // position/rotation packet (abs), sent by: 3Dc̘3Dconne SpaceMouse Wireless Receiver, vendor 0x256f product 0xc62f version 0x111
				switch(ev.code) {
				case 0:	// X
					current.axis[0] = ev.value;
					break;
				case 1:	// Y
					current.axis[1] = ev.value;
					break;
				case 2:	// Z
					current.axis[2] = ev.value;
					break;
				case 3:	// RY
					current.rotAxis[0] = ev.value;
					break;
				case 4:	// RY
					current.rotAxis[1] = ev.value;
					break;
				case 5:	// RZ
					current.rotAxis[2] = ev.value;
					break;
				default: 
					break;
				}
				break;
			default: // other
				break;
			}
		}
	}
}
