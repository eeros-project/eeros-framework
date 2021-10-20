#include <eeros/hal/HAL.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/hal/ODrive_UART.hpp>
#include <eeros/math/Matrix.hpp>


#include <iostream>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <iomanip>
#include <unistd.h>

# define FULL_STATE_CALIBRATION 3
# define CLOSED_LOOP_CONTROL 8
# define AXIS_STATE_IDLE 1

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::math;
using namespace slingload;

int nofaxis_odrive = 1;

ODrive_UART::ODrive_UART(std::string dev, int speed, int parity, int priority) : 
Thread(priority),
log(Logger::getLogger('P')) {
	// open communication
	openTty(dev, speed, parity);
	// initializations
	calibrate_motors();
	sleep(15); 					// TODO wait to be finished
	set_closed_loop_control();
	sleep(1); 					// TODO wait to be finished
		
	started = true;
// 	set_speed(0, 10000);
}

ODrive_UART::~ODrive_UART() {	
	for(int i=0; i<=nofaxis_odrive-1; i++){
		// Set speed to 0
		set_speed(i, 0);
		
		// Set axis to IDLE
		std::stringstream stream;
		stream << "w axis" << i << ".requested_state " << AXIS_STATE_IDLE << '\r'; 
		std::string str(stream.str());
		log.info() << "Axis IDLE " << i << " -> stream: " << str;

		int n = write(ttyFd, str.c_str(), str.length());
		if (n < 0) log.error() << "error " << errno << " set axes to idle -> write to ODrive device failed";
	}	
	
	running = false;
	closeTty();
}

void ODrive_UART::openTty(std::string dev, int speed, int parity) {
	log.info() << "Open ODrive TTY, port: " << dev;

	ttyFd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (ttyFd < 0) throw eeros::Fault("error opening tty device");
	set_interface_attributes(speed, parity); // set speed and parity
	log.info() << "ODrive interface now open";
}

int ODrive_UART::set_interface_attributes (int speed, int parity) {
	struct termios tty;
	memset (&tty, 0, sizeof tty);

	if (tcgetattr (ttyFd, &tty) != 0) {
		log.info() << "error " << errno << " from tcgetattr";
		return -1;
	}
	
	cfsetospeed (&tty, (speed_t)speed);
	cfsetispeed (&tty, (speed_t)speed);
	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars

	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
// 	tty.c_cc[VMIN]  = 0;            // read doesn't block
// 	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (ttyFd, TCSANOW, &tty) != 0) {
		log.error() << "error " << errno << " from tcsetattr";
		return -1;
	}
	return 0;
}

void ODrive_UART::closeTty() {
	close(ttyFd);
	log.info() << "ODrive interface now closed";
}

void ODrive_UART::calibrate_motors(){
	for(int i=0; i<=nofaxis_odrive-1; i++){
		std::stringstream stream;
		stream << "w axis" << i << ".requested_state " << FULL_STATE_CALIBRATION << '\r'; 
		std::string str(stream.str());
		log.info() << "Calibration " << i << " -> stream: " << str;

		int n = write(ttyFd, str.c_str(), str.length());
		if (n < 0) log.error() << "error " << errno << " calibrate_motors() -> write to ODrive device failed";
	}	
}

void ODrive_UART::set_closed_loop_control(){
	for(int i=0; i<=nofaxis_odrive-1; i++){
		std::stringstream stream;
		stream << "w axis" << i << ".requested_state " << CLOSED_LOOP_CONTROL << '\r'; 
		std::string str(stream.str());
		log.info() << "Set closed loop control " << i << " -> stream: " << str;

		int n = write(ttyFd, str.c_str(), str.length());
		if (n < 0) log.error() << "error " << errno << " set_closed_loop_control() -> write to ODrive device failed";
	}	
}

void ODrive_UART::set_speed(int motor, int speed){
	std::stringstream stream;
	stream << "v " << motor << " " << speed << '\r'; 
	std::string str(stream.str());

	int n = write(ttyFd, str.c_str(), str.length());
	if (n < 0) log.error() << "error " << errno << " set_speed() -> write to ODrive device failed";
}

void ODrive_UART::get_vel_from_encoder(int motor_id){
	// Request get_encoder_vel
	std::stringstream stream;
	stream << "r axis" << motor_id << ".encoder.vel_estimate" << '\r'; 
	std::string str(stream.str());

	int n = write(ttyFd, str.c_str(), str.length());
	if (n < 0) log.error() << "error " << errno << " get_measurements() -> write to ODrive device failed";
	
	// Read data
	uint8_t data;
	int size_data = 100;
	char tmpData[size_data];
	char* ptr = tmpData;

	int n_read;
	do {
		n_read = read(ttyFd, &data, 1);
		*ptr = data;
		ptr++;
	} while (data != 0x2e);
	ptr--;
	*ptr = 0;
	std::string str_read(tmpData);
	// log.info() << str_read;
	
	// Write data on variable
	if(motor_id == 0)
		encoder_vel0 = std::stod(str_read);
	else if(motor_id == 1)
		encoder_vel1 = std::stod(str_read); 
	
	do{
		n_read = read(ttyFd, &data, 1);
		if (n_read < 0) log.error() << "error " << errno << " read from ODrive device failed";
	} while (data != 0xd);
	
	n_read = read(ttyFd, ptr, 2);
}

void ODrive_UART::run() {
    while(!started);
    running = true;
	
	static int count = 0;
    while (running) {
		set_speed(0, 10000);
        get_vel_from_encoder(0);
        get_vel_from_encoder(1);
		
		count++;
		if(count % 100 == 0)
			std::cout << eeros::System::getTimeNs() << std::endl;
    }
    
}

double ODrive_UART::get_actual_vel(int motor_id){
	if(motor_id == 0)
		return encoder_vel0;
	else if(motor_id == 1)
		return encoder_vel1;
	else {
		log.error() << "Wrong motor ID. Encoder speed not valid";
		return 0;
	}
}
