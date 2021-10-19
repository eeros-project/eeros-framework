#ifndef ORG_EEROS_HAL_BAUMEROM70_HPP_
#define ORG_EEROS_HAL_BAUMEROM70_HPP_

#include <eeros/core/Runnable.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/math/Matrix.hpp>
#include "../external/libmodbus-3.1.6/src/modbus.h"    

#include <eeros/core/PeriodicCounter.hpp>

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
			explicit BaumerOM70(std::string dev, int port, int slave_id, int priority);
			
			/**
			* Destructs a Thread to get Baumer OM70 sensors data \n
			*/
			~BaumerOM70();
			
			float distance;
			float meas_rate;
			uint16_t signal_quality;
			uint32_t timestamp_us;
			
			/**
			* Gets data from Baumer OM70 sensor 
			* Is called by the run() method
			* @see run()
			*/
			void get_measurements();
			
			/**
			* Gets range of measurement limits set 
			*/
			void get_meas_range_limits();
			
			/**
			* Gets IP-Adress of Baumer OM70 sensor 
			*/
			void get_ip_adress();
			
			/**
			* Switches ON Laser of Baumer OM70 sensor 
			*/
			void switch_on_laser();
			
			/**
			* Switches OFF Laser of Baumer OM70 sensor 
			*/
			void switch_off_laser();
			
			/**
			* Gets distance measurement
			* 
			* @return distance
			*/
			float get_distance();

		private:
			eeros::PeriodicCounter pc; 
			eeros::logger::Logger log;
						
			/**
			* Runs methods for data acquisition from sensor
			* Calls get:measurements()
			* @see get_measurements()
			* 
			*/
			virtual void run();
			
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


