#ifndef ORG_EEROS_CONTROL_SBGELLIPSEAINPUT_HPP_
#define ORG_EEROS_CONTROL_SBGELLIPSEAINPUT_HPP_

#include <string>
#include <thread>
#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeors/hal/SBGEllipseA.hpp>

using namespace eeros::hal;
using namespace eeros::logger;


namespace eeros {
namespace control {

	class SBGEllipseAInput: public eeros::control::Block {
	public:
		/**
		* Constructs an input block to get data from SBG Elliipse-A IMU sensor. \n
		* Calls SBGEllipseAInput(std::string dev, int priority)
		*
		* @see  SBGEllipseAInput(std::string dev, int priority)
		* @param dev - string with device name (USB)
		* @param priority - execution priority or SBGEllipseA thread, to get sensors data
		*/
		SBGEllipseAInput(std::string dev, int priority = 5);

		/**
		* Disabling use of copy constructor because the block should never be copied unintentionally.
		*/
		SBGEllipseAInput(const SBGEllipseAInput& s) = delete; 
		
		/**
		* Gets input data from SBGEllipseA Thread and outputs them
		*/
		virtual void run();
		
        /**
		* Gets the output orientation, expressed in euler angles 
		* 
		* @return euler_out
		*/
		virtual eeros::control::Output<eeros::math::Vector3>& getOut_eulerAng();
		
        /**
		* Gets the output orientation, expressed in quaternions 
		* 
		* @return quaternion_out
		*/
		virtual eeros::control::Output<eeros::math::Vector4>& getOut_quaternion();
		
        /**
		* Gets the output linear accelerations ax, ay, az from accelerometer 
		* 
		* @return acc_out
		*/
		virtual eeros::control::Output<eeros::math::Vector3>& getOut_acc();
		
        /**
		* Gets the output angular velocities wx, wy, wz from gyro 
		* 
		* @return gyro_out
		*/
		virtual eeros::control::Output<eeros::math::Vector3>& getOut_gyro();
		
        /**
		* Gets the output timestamp of the sensor signal 
		* 
		* @return timestamp_out
		*/
		virtual eeros::control::Output<uint32_t>& getOut_timestamp();
		
	protected:
		eeros::control::Output<eeros::math::Vector3> euler_out, acc_out, gyro_out;
		eeros::control::Output<eeros::math::Vector4> quaternion_out;
		eeros::control::Output<uint32_t> timestamp_out;
		
		SBGEllipseA sbg;
		Logger log;
	};
};
}

#endif /* ORG_EEROS_CONTROL_SBGELLIPSEAINPUT_HPP_ */

