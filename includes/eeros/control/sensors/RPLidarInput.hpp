#ifndef ORG_EEROS_CONTROL_RPLIDAR_INPUT_HPP
#define ORG_EEROS_CONTROL_RPLIDAR_INPUT_HPP

#include <thread>
#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/hal/RPLidar.hpp> 

#define LASER_COUNT_MAX 380

namespace eeros {
namespace control {
	class RPLidarInput : public eeros::control::Blockio<0,2,eeros::math::Matrix<LASER_COUNT_MAX,1>>
	{        
        public:
			/**
			* Constructs an rplidar input block, whic outputs data received from a RPLidar thread \n
			* Calls RPLidarInput(std::string dev, int priority)
			*
			* @see  RPLidarInput(std::string dev, int priority)
			* @param dev - string with device name
			* @param priority - execution priority or RPLidar thread, to get sensors data
			*/
            RPLidarInput(std::string dev, int priority = 5);
            
			/**
			* Disabling use of copy constructor because the block should never be copied unintentionally.
			*/
			RPLidarInput(const RPLidarInput& s) = delete; 
			
			/**
			* Runs the filter algorithm.
			*
			* Gets input data from RPLidar Thread and outputs them
			* output[0] = angles
			* output[1] = ranges
			* output timestamp = system time
			*/
            virtual void run();
            
        private:                     
            eeros::hal::RPLidar rplidar;
			static const laser_count_max = 1000; // TODO right number, depending on resolution
    };
};
}

#endif /* ORG_EEROS_CONTROL_RPLIDAR_INPUT_HPP */
