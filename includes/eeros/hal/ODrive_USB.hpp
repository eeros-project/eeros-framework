#ifndef ORG_EEROS_HAL_ODRIVE_USB_HP
#define ORG_EEROS_HAL_ODRIVE_USB_HP

#include <atomic>
#include <iostream>
#include <sstream>
#include <getopt.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <cstring>
#include <string>
#include <vector>
#include <libusb-1.0/libusb.h>
#include <endian.h>
#include <jsoncpp/json/json.h>

#include <eeros/core/Runnable.hpp>
#include <eeros/core/PeriodicCounter.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>

#include "../external/odrive_ost/include/odrive/odriveEP.hpp"
#include "../constants.hpp"

namespace eeros {
namespace hal {
	typedef std::vector<uint8_t> commBuffer;

    class ODrive_USB : public eeros::Thread {
    
        public: 
			/**
			* Constructs a Thread to communicate with ODrive Device over USB \n
			* Calls ODrive_USB(uint64_t odrive_serialnr, float enc_ticks, int priority, bool first_drive)
			*
			* @param odrive_serialnr - device serial number
			* @param enc_ticks - encoder ticks of motors connected
			* @param priority - execution priority of this thread
			* @param first_drive - one drive must be set as the first one for reset tasks 
			*/
            explicit ODrive_USB(uint64_t odrive_serialnr, float enc_ticks, int priority, bool first_drive);
					
			/**
			* Destructs a Thread to get and send data from and to odrive \n
			*/
            ~ODrive_USB();
			
			odrive::ODriveEP* ep;
			Json::Value json;
			int init(uint64_t odrive_serialnr);
			int getJson();
			float encoderTicksPerRad;
			
			double get_encoder_vel(int motor);
			void set_(int motor);
			void set_ref_vel(int motor, float speed_rad);
			void enable_drives();
			void disable_drives();
			void do_start_calibration();
			bool is_endstop_active();
			bool is_calibrated();
			void calibrate_motors(); 
			
			/**
			* Returns current setpoint of one motor  \n
			* @param motor - motor id (0 or 1)
			*/
			double get_current_setpoint(int motor); // TEST
			
			/**
			* Returns measured current of one motor  \n
			* @param motor - motor id (0 or 1)
			*/
			double get_current_measured(int motor); // TEST
			
        private:
            eeros::PeriodicCounter pc; 
            eeros::logger::Logger log;
  
			libusb_context* libusbContext;
			
            bool started = false;
			bool running = false;
			bool first_drive = false;
			bool start_calibration_cmd = false;
			bool calibration_done = false;
			
			uint64_t odrive_serialnr;
			int nof_motors;
			float enc_ticks;
			
			std::atomic<double> encoder_vel0;
			std::atomic<double> encoder_vel1;
			std::atomic<float> ref_vel0;
			std::atomic<float> ref_vel1;
			std::atomic<bool> endpoint_state0;
			std::atomic<bool> endpoint_state1;
			
			std::atomic<double> iq_setpoint0;
			std::atomic<double> iq_setpoint1;
			std::atomic<double> iq_measured0;
			std::atomic<double> iq_measured1;
			
			/**
			* Configures odrive: clears errors, sets watchdog, endpoints, control mode, gains  \n
			*/
			void config_odrive();
			/**
			* Sets control mode to velocity  \n
			*/
			void set_velocity_ctrl_mode();
			/**
			* Sets controller gains values  \n
			*/
			void set_controller_gains();
			/**
			* Enables watchdog signal \n
			*/
			void enable_watchdog();
			/**
			* Disables watchdog signal \n
			*/
			void disable_watchdog();
			/**
			* Returns velocity values of motors connected to odrive \n
			*/
			void get_vel_from_odrive();
			/**
			* Sets velocity setpoints of motors connected to odrive \n
			*/
			void set_vel_to_odrive();
			/**
			* Returns endpoint state from odrive \n
			*/
			void get_endpoint_state_from_odrive();
			/**
			* Returns current value from odrive \n
			*/
			void get_current_from_odrive();
			
			/**
			* Main run method. Gets measurements and sends setpoints \n
			*/
			virtual void run();
    };
}
}

#endif /* ORG_EEROS_HAL_ODRIVE_USB_HP */
