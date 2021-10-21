#ifndef ORG_EEROS_HAL_BAUMEROM70_HPP_
#define ORG_EEROS_HAL_BAUMEROM70_HPP_

#include <eeros/core/Runnable.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/PeriodicCounter.hpp>
#include "../external/libmodbus-3.1.6/src/modbus.h"    

// TODO check and delete
// #include <iostream>
// #include <unistd.h>
// #include <chrono>
// #include <thread>
// #include <eeros/logger/Logger.hpp>
// #include <eeros/logger/StreamLogWriter.hpp>
// #include "../external/libmodbus-3.1.6/src/modbus-tcp.h"
// 
// #include <bits/stdc++.h> // TODO delete after clock test
// 
// #include <eeros/hal/BaumerOM70.hpp>
// ... end TODO 

// * Read discrete inputs (FC02)
// * -> ...

// * Input Register: (Manual S. 51)
// *   Read input registers (FC04) -> all are 'read registers'
// * -> modbus_read_input_registers() 

// * Holding function: (Manual S. 45)
// *   Read holding registers (FC03)
// *   Write single holding register (FC06)
// *   Write multiple holding registers (FC16)
// * -> modbus_read_registers(), 
// TODO check modbus_write_registers() modbus_write_register()


namespace eeros {
namespace hal {
	/**
	* This class is part of the hardware abstraction layer. 
	* It is used by \ref eeros::control::BaumerOM70Input class. 
	* Do not use it directly.
	*
	*/
	class BaumerOM70 : public eeros::Thread {
	
		public: 
			/**
			* Constructs a Thread to get Baumer OM70 sensors data \n
			* Calls BaumerOM70(std::string dev, int port, int slave_id, int priority)
			*
			* @see BaumerOM70(std::string dev, int port, int slave_id, int priority)
			* @param dev - string with device name
			* @param port - port for sensor data read (modbus interface)
			* @param slave - sensor slave number (modbus interface)
			* @param priority - execution priority or BaumerOM70 thread, to get sensors data
			*/
			explicit BaumerOM70(std::string dev, int port, int slave_id, int priority) : 
			Thread(priority),
			log(eeros::logger::Logger::getLogger()) 
			{       
				slaveId = slave_id;
				
				// Create new tcp connection
				ctx = modbus_new_tcp(dev.c_str(), port);
				
				// Set slave
				modbus_set_slave(ctx, slave_id);
				
				log.info() << "Baumer OM70, Modbus new TCP connection " << dev.c_str() << ", slaveId " << slaveId;
				
				// Connect
				auto connect_output = modbus_connect(ctx);
				
				if (connect_output == -1) {
					log.info() << "Baumer OM70, Modbus connection failed: " << modbus_strerror(errno);
					modbus_free(ctx);
					return;
				}
				else{
					log.info() << "Baumer OM70, Modbus connection successfull"; //: " << connect_output;
				}
				
				started = true; 
			}

			
			/**
			* Destructs a Thread to get Baumer OM70 sensors data \n
			*/
			~BaumerOM70() {
				running = false; 
				join(); 
				
				modbus_close(ctx);
				modbus_free(ctx);
			}
			
			float distance;
			float meas_rate;
			uint16_t signal_quality;
			uint32_t timestamp_us;
			
			/**
			* Gets data from Baumer OM70 sensor 
			* Is called by the run() method
			* @see run()
			*/
			void get_measurements() {    
				time_t start, end;
				pc.tick();
				
				start = clock();
				rc = modbus_read_input_registers(ctx, 200, 17, tab_reg);
				end = clock();
				
				pc.tock();
				
				// Distance [mm]
				uint32_t* ptr_i = (uint32_t*)&tab_reg[3];
				float* ptr_f = reinterpret_cast<float*>(ptr_i);
				
				distance = (*ptr_f)*0.001; // from [mm] to [m]
				
				// Measurement rate [Hz]
				uint32_t* ptr_i_rate = (uint32_t*)&tab_reg[5];
				float* ptr_rate = reinterpret_cast<float*>(ptr_i_rate);
				
				meas_rate = *ptr_rate;
				
				// Timestamp [us]
				uint32_t* ptr_i_stamp = (uint32_t*)&tab_reg[15];
				timestamp_us = *ptr_i_stamp;
			}
			
			/**
			* Gets range of measurement limits set 
			*/
			void get_meas_range_limits(){
				rc = modbus_read_input_registers(ctx, 150, 6, tab_reg);
			}
			
			/**
			* Gets IP-Adress of Baumer OM70 sensor 
			*/
			void get_ip_adress(){
				rc = modbus_read_input_registers(ctx, 100, 6, tab_reg); // Ethernet configuration
				uint32_t* ptr_i = (uint32_t*)&tab_reg[0];
				float* ptr_f = reinterpret_cast<float*>(ptr_i); // TODO check to have string
			}
			
			/**
			* Switches ON Laser of Baumer OM70 sensor 
			*/
			void switch_on_laser(){
				log.info() << "Switch on";
				const uint16_t on = 1;
				rc = modbus_write_register(ctx, 410, on); // TODO write error Illegal data value
				
				if (rc == -1) {
					log.error() << "write error " << modbus_strerror(errno);
				}
				
				rc = modbus_read_registers(ctx, 410, 1, tab_reg);
				if (rc == -1)
					log.error() << "read error " << modbus_strerror(errno);
				else
					log.info() << tab_reg[0];
			}
			
			/**
			* Switches OFF Laser of Baumer OM70 sensor 
			*/
			void switch_off_laser(){
				const uint16_t off = 0;
				rc = modbus_write_register(ctx, 410, off); // TODO write error Illegal data value
				
				if (rc == -1) {
					log.error() << "write error " << modbus_strerror(errno);
				}
				
				rc = modbus_read_registers(ctx, 410, 1, tab_reg);
				if (rc == -1)
					log.error() << "read error " << modbus_strerror(errno);
				else
					log.info() << tab_reg[0];
			}
			
			/**
			* Gets distance measurement
			* 
			* @return distance
			*/
			float get_distance(){
				return distance;
			}

		private:
			eeros::PeriodicCounter pc; 
			eeros::logger::Logger log;
						
			/**
			* Runs methods for data acquisition from sensor
			* Calls get:measurements()
			* @see get_measurements()
			* 
			*/
			virtual void run() {
				while(!started);
				running = true;

				while (running) {
					get_measurements();
				}
			}
			
			volatile bool started = false;
			volatile bool running = false;
			
			int slaveId;
			modbus_t *ctx;
			uint16_t tab_reg[32];
			
			int rc;
			int i;
	};
};
}

#endif /* ORG_EEROS_HAL_BAUMEROM70_HPP_ */


