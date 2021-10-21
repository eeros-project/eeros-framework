#ifndef ORG_EEROS_HAL_REALSENSE_T265_HPP
#define ORG_EEROS_HAL_REALSENSE_T265_HPP

#include <eeros/core/Runnable.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/math/Matrix.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/h/rs_types.h>

// #include <iostream>
// #include <unistd.h>
// #include <chrono>
// #include <cstring>
// #include <eeros/logger/Logger.hpp>
// #include <eeros/logger/StreamLogWriter.hpp>
// #include <eeros/hal/RealsenseT265.hpp>
// 
// using namespace eeros::logger;
// using namespace eeros::math;
// using namespace eeros::hal;

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
		explicit RealsenseT265(std::string dev, int priority) : 
			Thread(priority),
			log(eeros::logger::Logger::getLogger('P')){
				started = false;
				
				cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);  // Add pose stream
				pipe.start(cfg);                                      // Start pipeline with chosen configuration
				
				started = true;
			}
		
		/**
		* Destructs a Thread to get RealsenseT265 sensors data \n
		*/
		~RealsenseT265() { 
			running = false;
			join();
		}
		
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
		virtual void run(){
			while(!started);
			running = true;
			
			while (running) {
				// Wait for the next set of frames from the camera
				auto frames = pipe.wait_for_frames();
				// Get a frame from the pose stream
				auto f = frames.first_or_default(RS2_STREAM_POSE);
				// Cast the frame to pose_frame and get its data
				auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
				
				// outputs for eeros
				translation          << pose_data.translation.x, pose_data.translation.y, pose_data.translation.z;
				velocity             << pose_data.velocity.x, pose_data.velocity.y, pose_data.velocity.z;
				acceleration         << pose_data.acceleration.x, pose_data.acceleration.y, pose_data.acceleration.z;
				quaternion           << pose_data.rotation.w, pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.z;
				angular_velocity     << pose_data.angular_velocity.x, pose_data.angular_velocity.y, pose_data.angular_velocity.z;
				angular_acceleration << pose_data.angular_acceleration.x, pose_data.angular_acceleration.y, pose_data.angular_acceleration.z;
				
				// Calculate current transformation matrix
				float r[16];
				calc_transform(pose_data, r);
				// From the matrix we found, get the new location point
				rs2_vector tr{ r[12], r[13], r[14] };
				// Create a new point to be added to the trajectory
				tracked_point p{ tr , pose_data.tracker_confidence };        
			}
		}
		volatile bool started;
		volatile bool running;
		
		rs2::pipeline pipe;  // Declare RealSense pipeline, encapsulating the actual device and sensors
		rs2::config cfg;     // Create a configuration for configuring the pipeline with a non default profile
		
		/**
		* Calculates transformation matrix based on pose data from the device 
		* Is called by the run() method
		* @see run()
		*/
		void calc_transform(rs2_pose& pose_data, float mat[16]) {
			auto q = pose_data.rotation;
			auto t = pose_data.translation;
			// Set the matrix as column-major for convenient work with OpenGL and rotate by 180 degress (by negating 1st and 3rd columns)
			mat[0] = -(1 - 2 * q.y*q.y - 2 * q.z*q.z); mat[4] = 2 * q.x*q.y - 2 * q.z*q.w;     mat[8] = -(2 * q.x*q.z + 2 * q.y*q.w);      mat[12] = t.x;
			mat[1] = -(2 * q.x*q.y + 2 * q.z*q.w);     mat[5] = 1 - 2 * q.x*q.x - 2 * q.z*q.z; mat[9] = -(2 * q.y*q.z - 2 * q.x*q.w);      mat[13] = t.y;
			mat[2] = -(2 * q.x*q.z - 2 * q.y*q.w);     mat[6] = 2 * q.y*q.z + 2 * q.x*q.w;     mat[10] = -(1 - 2 * q.x*q.x - 2 * q.y*q.y); mat[14] = t.z;
			mat[3] = 0.0f;                             mat[7] = 0.0f;                          mat[11] = 0.0f;                             mat[15] = 1.0f;
		}
		
/*		void add_to_trajectory(tracked_point& p);*/    
	};
};
}

#endif /* ORG_EEROS_HAL_REALSENSE_T265_HPP */
