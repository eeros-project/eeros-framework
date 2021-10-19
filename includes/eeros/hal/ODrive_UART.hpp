#ifndef SLINGLOAD_ODRIVE_UART_HPP
#define SLINGLOAD_ODRIVE_UART_HPP

#include <eeros/core/Runnable.hpp>
#include <eeros/core/PeriodicCounter.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
// #include <eeros/math/Matrix.hpp>

#include <vector>
#include <atomic>

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
            explicit ODrive_UART(std::string dev, int speed, int parity, int priority);
				
			/**
			* Destructs a Thread to get and send data from and to odrive \n
			*/
            ~ODrive_UART();
            
			/**
			* Opens tty communication \n
			*/
			void openTty(std::string portname, int speed, int parity);
			/**
			* Closes tty communication \n
			*/
			void closeTty();
			
			/**
			* Returns actual velocity of one motor \n
			* @param motor_id - motor id (0 or 1)
			*/
			double get_actual_vel(int motor_id);
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
			int set_interface_attributes(int speed, int parity);
			/**
			* Perform motors calibration \n
			*/
			void calibrate_motors();
			/**
			* Set control to closed loop \n
			*/
			void set_closed_loop_control();
			/**
			* Returns velocity measured by encoder \n
			*/
			void get_vel_from_encoder(int motor);
			
			/**
			 * Main run method. Get measurements from encoders and set setpoints \n
			 */
			virtual void run();
			
            bool started = false;
			bool running;
			int ttyFd;
			
			std::atomic<double> encoder_vel0;
			std::atomic<double> encoder_vel1;
    };
}

#endif /* SLINGLOAD_ODRIVE_UART_HPP */
