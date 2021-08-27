#ifndef ORG_EEROS_HAL_REALSENSE_T265_HPP
#define ORG_EEROS_HAL_REALSENSE_T265_HPP

#include <eeros/core/Runnable.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/math/Matrix.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/h/rs_types.h>

namespace eeros {
namespace hal {
	/**
	* This class is part of the hardware abstraction layer. 
	* It is used by \ref eeros::control::RealsenseT265Input class. 
	* Do not use it directly.
	*
	*/
	class RealsenseT265 : public eeros::Thread {
		
	public:
		// Struct to store trajectory points
		struct tracked_point {
			rs2_vector point;
			unsigned int confidence;
		};
		
		/**
		* Constructs a Thread to get Realsense Tracking T265 sensors data \n
		* Calls RealsenseT265(std::string dev, int priority)
		*
		* @see RealsenseT265(std::string dev, int priority)
		* @param dev - string with device name (USB3)
		* @param priority - execution priority or RealsenseT265 thread, to get sensors data
		*/
		explicit RealsenseT265(std::string dev, int priority); 
		
		/**
		* Destructs a Thread to get RealsenseT265 sensors data \n
		*/
		~RealsenseT265();
		
		eeros::math::Vector3 translation;
		eeros::math::Vector3 velocity;
		eeros::math::Vector3 acceleration;
		eeros::math::Vector4 quaternion;
		eeros::math::Vector3 angular_velocity;
		eeros::math::Vector3 angular_acceleration;

	private:
		eeros::logger::Logger log;
				
		/**
		* Runs methods for data acquisition from sensor
		* Gets data, performs scaling, saves tracked_point information on a vector
		* @see calc_transform
		* @see tracked_point
		* 
		*/
		virtual void run();
		volatile bool started;
		volatile bool running;
		
		rs2::pipeline pipe;  // Declare RealSense pipeline, encapsulating the actual device and sensors
		rs2::config cfg;     // Create a configuration for configuring the pipeline with a non default profile
		
		/**
		* Calculates transformation matrix based on pose data from the device 
		* Is called by the run() method
		* @see run()
		*/
		void calc_transform(rs2_pose& pose_data, float mat[16]);
		
/*		void add_to_trajectory(tracked_point& p);*/    
	};
};
}

#endif /* ORG_EEROS_HAL_REALSENSE_T265_HPP */
