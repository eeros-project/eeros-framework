#ifndef ORG_EEROS_HAL_RPLIDAR_HPP_
#define ORG_EEROS_HAL_RPLIDAR_HPP_

#include <eeros/core/Runnable.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/math/Matrix.hpp>
#include <rplidar_sdk/sdk/sdk/include/rplidar.h>

#define LASER_COUNT_MAX 380

using namespace rp::standalone::rplidar;

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
		explicit RPLidar(std::string dev, int priority);
		
		/**
		* Destructs a Thread to get Baumer OM70 sensors data \n
		*/
		~RPLidar();

		/**
		* Gets scanning frequency and returns it to the user
		* 
		* @return frequency
		*/
		float get_scan_frequency(); 
		
		/**
		* Gets angles where a range has been measured
		* 
		* @return laser_angles
		*/
		eeros::math::Vector<LASER_COUNT_MAX,double> get_angles();  // TODO richtige Groesse angeben
		
		/**
		* Gets range measurements
		* 
		* @return laser_ranges
		*/
		eeros::math::Vector<LASER_COUNT_MAX,double> get_ranges();
		
		/**
		* Gets range intensities
		* 
		* @return laser_intensities
		*/
		eeros::math::Vector<LASER_COUNT_MAX,double> get_intensities();
		
		RPlidarDriver * laser_drv;
		RplidarScanMode scan_mode;
		
		int buffer_size;
		
	private:		
		/**
		* Runs methods for data acquisition from sensor
		* Gets data, performs scaling, saves angles and range data on a vector
		* 
		*/
		virtual void run();
		volatile bool started;
		volatile bool running;
		
// 		static const laser_count_max = LASER_COUNT_MAX; // TODO right number, depending on resolution
			
		eeros::logger::Logger log;
		eeros::math::Vector<LASER_COUNT_MAX,double> laser_angles, laser_ranges, laser_intensities; 
	};
};
}

#endif /* ORG_EEROS_HAL_RPLIDAR_HPP_ */
