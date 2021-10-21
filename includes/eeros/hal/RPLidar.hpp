#ifndef ORG_EEROS_HAL_RPLIDAR_HPP_
#define ORG_EEROS_HAL_RPLIDAR_HPP_

#include <eeros/core/Runnable.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/math/Matrix.hpp>
#include <rplidar.h>

#define LASER_COUNT_MAX 380

// #include <iostream>
// #include <unistd.h>
// #include <chrono>
// #include <eeros/logger/Logger.hpp>
// #include <eeros/logger/StreamLogWriter.hpp>
// 
// #include <eeros/hal/RPLidar.hpp>
// 
// using namespace eeros::logger;
// using namespace eeros::hal;
// using namespace eeros::math;
// using namespace rp::standalone::rplidar;

using namespace rp::standalone::rplidar;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

namespace eeros {
namespace hal {
	/**
	* This class is part of the hardware abstraction layer. 
	* It is used by \ref eeros::control::RPLidarInput class. 
	* Do not use it directly.
	*
	*/
	class RPLidar : public eeros::Thread {
		
	public:
		/**
		* Constructs a Thread to get RPLidar (Laserscanner) sensors data \n
		* Calls RPLidar(std::string dev, int priority)
		*
		* @see RPLidar(std::string dev, int priority)
		* @param dev - string with device name (USB)
		* @param priority - execution priority or BaumerOM70 thread, to get sensors data
		*/
		explicit RPLidar(std::string dev, int priority) :
		Thread(priority),
		log(eeros::logger::Logger::getLogger('P')) {
			// TODO check if used
			buffer_size = LASER_COUNT_MAX;
			
			// Create driver instance
			laser_drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
			
			if (!laser_drv){
				log.error() << "RPLidar Create driver instance: insufficent memory, exit";
				return;
			}
			
			// Connect laser
			rplidar_response_device_info_t devinfo;
			u_result op_result;
			u_result res = laser_drv->connect(dev.c_str(), 115200);

			if (IS_OK(res)) {    
				// Print out the device serial number, firmware and hardware version number
				printf("RPLidar S/N: ");
				for (int pos = 0; pos < 16 ;++pos) {
					printf("%02X", devinfo.serialnum[pos]);
				}
				printf("\n"
						"RPLidar Firmware Ver: %d.%02d\n"
						"RPLidar Hardware Rev: %d\n"
						, devinfo.firmware_version>>8
						, devinfo.firmware_version & 0xFF
						, (int)devinfo.hardware_version);

				//  Check health
				u_result op_result;
				rplidar_response_device_health_t healthinfo;

				op_result = laser_drv->getHealth(healthinfo);
				if (IS_OK(op_result)) { 
					log.info() <<"RPLidar health status : " << healthinfo.status;
					if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
						log.error() << "RPLidar Error, rplidar internal error detected. Please reboot the device to retry";
					} 
				} 
				else {
					log.error() << "RPLidar Error, cannot retrieve the lidar health code: " << op_result;
				}
			}
			else{
				log.error() << "RPLidar Failed to connect to LIDAR";
				return;
			}
			
			// Start laser motor
			laser_drv->startMotor();
			
			// Start laser scan
			laser_drv->startScan(false, true, 0, &scan_mode);  
			
			// Start 
			started = true;
		}

		
		/**
		* Destructs a Thread to get Baumer OM70 sensors data \n
		*/
		~RPLidar() { 
			running = false;
			join();
			
			laser_drv->stop();
			laser_drv->stopMotor();
			laser_drv->disconnect();
			RPlidarDriver::DisposeDriver(laser_drv);
		}

		/**
		* Gets scanning frequency and returns it to the user
		* 
		* @return frequency
		*/
		float get_scan_frequency(){
			float frequency;
			laser_drv->getFrequency(scan_mode, buffer_size, frequency);
			
			return frequency;
		}
		
		/**
		* Gets angles where a range has been measured
		* 
		* @return laser_angles
		*/
		eeros::math::Vector<LASER_COUNT_MAX,double> get_angles(){
			return laser_angles;
		}
		/**
		* Gets range measurements
		* 
		* @return laser_ranges
		*/
		eeros::math::Vector<LASER_COUNT_MAX,double> get_ranges(){
			return laser_ranges;
		}
		
		/**
		* Gets range intensities
		* 
		* @return laser_intensities
		*/
		eeros::math::Vector<LASER_COUNT_MAX,double> get_intensities(){
			return laser_intensities;
		}
		
		RPlidarDriver * laser_drv;
		RplidarScanMode scan_mode;
		
		int buffer_size;
		
	private:		
		/**
		* Runs methods for data acquisition from sensor
		* Gets data, performs scaling, saves angles and range data on a vector
		* 
		*/
		virtual void run() {
			while(!started);
			running = true;

			while (running) {
				u_result ans;
				
				rplidar_response_measurement_node_hq_t nodes[LASER_COUNT_MAX];
				size_t count = _countof(nodes);

				// fetech extactly one 0-360 degrees' scan
				ans = laser_drv->grabScanDataHq(nodes, count);
				buffer_size = count; 

				// printf("final count = %d\n", (int)count);
				
				if (IS_OK(ans)){  
					laser_drv->ascendScanData(nodes, count);
					
					// Set angles
					int start_node = 0, end_node = 0;
					int i = 0;
					// find the first valid node and last valid node
					while (nodes[i++].dist_mm_q2 == 0);
					start_node = i-1;
					i = count -1;
					while (nodes[i--].dist_mm_q2 == 0);
					end_node = i+1;
					
					// Set ranges and intensities
					laser_ranges.zero();
					laser_intensities.zero();
					
					for (int pos = 0; pos < (int)count ; ++pos) {               
						float read_value = (float) nodes[pos].dist_mm_q2/4.0f/1000;
						
						if (fabs(read_value) < 0.01)
							laser_ranges[pos] = std::numeric_limits<float>::infinity();
						else
							laser_ranges[pos] = read_value;
						
						laser_intensities[pos] = (float) (nodes[pos].quality >> 2);
						
						laser_angles[pos] = (nodes[pos].angle_z_q14 * 90.f / 16384.f) * 3.1415926535/180;
					}
					
					// Set to infinite all elements of the fixed-sized vector for control system
					for (int pos = (int)count; pos < LASER_COUNT_MAX; ++pos) {  
						laser_ranges[pos] = std::numeric_limits<float>::infinity();
						laser_intensities[pos] = std::numeric_limits<float>::infinity();
						laser_angles[pos] = std::numeric_limits<float>::infinity();
					}
				} 
				else if(ans == RESULT_OPERATION_TIMEOUT){
					log.error() << "RPLidar Laser Timeout! error code: " << ans;
				}
				else {
					log.error() << "RPLidar Laser error code: " << ans;
				}
			}
		}
		
		volatile bool started;
		volatile bool running;
		
		eeros::logger::Logger log;
		eeros::math::Vector<LASER_COUNT_MAX,double> laser_angles, laser_ranges, laser_intensities; 
	};
};
}

#endif /* ORG_EEROS_HAL_RPLIDAR_HPP_ */
