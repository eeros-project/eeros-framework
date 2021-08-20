#ifndef ORG_EEROS_CONTROL_REALSENSET265_INPUT_HPP
#define ORG_EEROS_CONTROL_REALSENSET265_INPUT_HPP

#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/PeriodicCounter.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/hal/RealsenseT265.hpp>

namespace eeros {
namespace control {
	class RealsenseT265Input : public eeros::control::Block
	{
	public:
		/**
		* Constructs an input block to get data from Realsense Tracking T265 sensor. \n
		* Calls RealsenseT265Input(std::string dev, int priority)
		*
		* @see  RealsenseT265Input(std::string dev, int priority)
		* @param dev - string with device name (USB 3.0)
		* @param priority - execution priority or BaumerOM70 thread, to get sensors data
		*/
		RealsenseT265Input(std::string dev, int priority = 5);
		
		/**
		* Disabling use of copy constructor because the block should never be copied unintentionally.
		*/
		RealsenseT265Input(const RealsenseT265Input& s) = delete; 
		
		
		/**
		* Gets input data from Realsense Tracking T265 Thread and outputs them
		*/
		virtual void run();
		
        /**
		* Gets the output translation x, y, z of tracking system, referred to position at data acquisition start
		* 
		* @return out_translation
		*/
        virtual eeros::control::Output<eeros::math::Vector3>& getOut_translation();
        /**
		* Gets the output linear velocity vx, vy, vz  of tracking system
		* 
		* @return out_velocity
		*/
		virtual eeros::control::Output<eeros::math::Vector3>& getOut_velocity();
       
		/**
		* Gets the output linear acceleration ax, ay, az of tracking system
		* 
		* @return out_acceleration
		*/
		virtual eeros::control::Output<eeros::math::Vector3>& getOut_acceleration();
        
		/**
		* Gets the output angular velocity wx, wy, wz  of tracking system
		* 
		* @return out_angular_velocity
		*/
		virtual eeros::control::Output<eeros::math::Vector3>& getOut_angularVel();
        
		/**
		* Gets the output angular acceleration alx, aly, alz of tracking system 
		* 
		* @return out_angular_acceleration
		*/
		virtual eeros::control::Output<eeros::math::Vector3>& getOut_angularAcc();
        
		/**
		* Gets the output orientation of the tracking system, expressed in quaternions
		* 
		* @return out_quaternion
		*/
		virtual eeros::control::Output<eeros::math::Vector4>& getOut_quaternion();
		
	private:
        eeros::control::Output<eeros::math::Vector3> out_translation, out_velocity, out_acceleration, out_angular_velocity, out_angular_acceleration;
        eeros::control::Output<eeros::math::Vector4> out_quaternion;
        
		RS_T265 t265;
        eeros::PeriodicCounter pc;
	};
};
}

#endif /* ORG_EEROS_CONTROL_REALSENSET265_INPUT_HPP */

