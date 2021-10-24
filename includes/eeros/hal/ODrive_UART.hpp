#ifndef SLINGLOAD_ODRIVE_UART_HPP
#define SLINGLOAD_ODRIVE_UART_HPP

#include <eeros/core/Runnable.hpp>
#include <eeros/core/PeriodicCounter.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
// #include <eeros/math/Matrix.hpp>

// TODO check and delete
// #include <eeros/hal/HAL.hpp>
// #include <eeros/core/Fault.hpp>
// #include <eeros/math/Matrix.hpp>
// #include <iostream>
// #include <errno.h>
// #include <fcntl.h> 
// #include <string.h>
// #include <termios.h>
// #include <iomanip>
// #include <unistd.h>

#include <vector>
#include <atomic>

# define FULL_STATE_CALIBRATION 3
# define CLOSED_LOOP_CONTROL 8
# define AXIS_STATE_IDLE 1

int nofaxis_odrive = 1;

namespace slingload
{
    class ODrive_UART : public eeros::Thread {
    
        public: 
			/**
			* Constructs a Thread to communicate with ODrive Device over UART \n
			* Calls ODrive_UART(std::string dev, int speed, int parity, int priority)
			*
			* @param dev - string with device id
			* @param speed - communication speed of UART
			* @param parity - parity bit set or not
			* @param priority - execution priority of this thread
			*/
            explicit ODrive_UART(std::string dev, int speed, int parity, int priority): 
				Thread(priority),
				log(Logger::getLogger('P')) {
					// open communication
					openTty(dev, speed, parity);
					// initializations
					calibrate_motors();
					sleep(15); 					// wait to be finished..
					set_closed_loop_control();
					sleep(1); 					// wait to be finished..
						
					started = true;
				// 	set_speed(0, 10000);
				}
				
			/**
			* Destructs a Thread to get and send data from and to odrive \n
			*/
            ~ODrive_UART(){
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
						
			/**
			* Opens tty communication \n
			*/
			void openTty(std::string portname, int speed, int parity) {
				log.info() << "Open ODrive TTY, port: " << dev;

				ttyFd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
				if (ttyFd < 0) throw eeros::Fault("error opening tty device");
				set_interface_attributes(speed, parity); // set speed and parity
				log.info() << "ODrive interface now open";
			}
			/**
			* Closes tty communication \n
			*/
			void closeTty(){
				close(ttyFd);
				log.info() << "ODrive interface now closed";
			}
			
			/**
			* Returns actual velocity of one motor \n
			* @param motor_id - motor id (0 or 1)
			*/
			double get_actual_vel(int motor_id){
				if(motor_id == 0)
					return encoder_vel0;
				else if(motor_id == 1)
					return encoder_vel1;
				else {
					log.error() << "Wrong motor ID. Encoder speed not valid";
					return 0;
				}
			}
			/**
			* Sets speed setpoint \n
			* @param motor - motor id (0 or 1)
			* @param speed - speed to be set to the motor
			*/
			void set_speed(int motor, int speed);
			
        private:
            eeros::PeriodicCounter pc; 
            eeros::logger::Logger log;
			
			/**
			* Sets attributes to communication intefrace \n
			* @param speed - communication speed
			* @param parity - parity bit set or not
			*/
			int set_interface_attributes(int speed, int parity) {
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
			/**
			* Perform motors calibration \n
			*/
			void calibrate_motors(){
				for(int i=0; i<=nofaxis_odrive-1; i++){
					std::stringstream stream;
					stream << "w axis" << i << ".requested_state " << FULL_STATE_CALIBRATION << '\r'; 
					std::string str(stream.str());
					log.info() << "Calibration " << i << " -> stream: " << str;

					int n = write(ttyFd, str.c_str(), str.length());
					if (n < 0) log.error() << "error " << errno << " calibrate_motors() -> write to ODrive device failed";
				}	
			}
			/**
			* Set control to closed loop \n
			*/
			void set_closed_loop_control(){
				for(int i=0; i<=nofaxis_odrive-1; i++){
					std::stringstream stream;
					stream << "w axis" << i << ".requested_state " << CLOSED_LOOP_CONTROL << '\r'; 
					std::string str(stream.str());
					log.info() << "Set closed loop control " << i << " -> stream: " << str;

					int n = write(ttyFd, str.c_str(), str.length());
					if (n < 0) log.error() << "error " << errno << " set_closed_loop_control() -> write to ODrive device failed";
				}	
			}
			/**
			* Returns velocity measured by encoder \n
			*/
			void get_vel_from_encoder(int motor_id){
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
			
			/**
			 * Main run method. Get measurements from encoders and set setpoints \n
			 */
			virtual void run() {
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
			
            bool started = false;
			bool running;
			int ttyFd;
			
			std::atomic<double> encoder_vel0;
			std::atomic<double> encoder_vel1;
    };
}

#endif /* SLINGLOAD_ODRIVE_UART_HPP */
