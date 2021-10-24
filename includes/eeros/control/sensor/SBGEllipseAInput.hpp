#ifndef ORG_EEROS_CONTROL_SBGELLIPSEAINPUT_HPP_
#define ORG_EEROS_CONTROL_SBGELLIPSEAINPUT_HPP_

#include <string>
#include <thread>
#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/hal/SBGEllipseA.hpp>

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
		SBGEllipseAInput(std::string dev, int priority = 5) : 
		sbg(dev, priority),
		log(Logger::getLogger()) 
		{}

		/**
		* Disabling use of copy constructor because the block should never be copied unintentionally.
		*/
		SBGEllipseAInput(const SBGEllipseAInput& s) = delete; 
		
		/**
		* Gets input data from SBGEllipseA Thread and outputs them
		*/
		virtual void run(){
// 			euler_out.getSignal().setValue(SBGEllipseA::data_euler);
// 			quaternion_out.getSignal().setValue(SBGEllipseA::data_quat);
// 			acc_out.getSignal().setValue(SBGEllipseA::data_acc);
// 			gyro_out.getSignal().setValue(SBGEllipseA::data_gyro);
// 			timestamp_out.getSignal().setValue(SBGEllipseA::timestamp_euler);
				
			// Timestamps
			uint64_t ts = eeros::System::getTimeNs();
			euler_out.getSignal().setTimestamp(ts);
			quaternion_out.getSignal().setTimestamp(ts);
			acc_out.getSignal().setTimestamp(ts);
			gyro_out.getSignal().setTimestamp(ts);
			timestamp_out.getSignal().setTimestamp(ts);
		}
		
        /**
		* Gets the output orientation, expressed in euler angles 
		* 
		* @return euler_out
		*/
		virtual eeros::control::Output<eeros::math::Vector3>& getOut_eulerAng() {
			return euler_out;
		}
		
        /**
		* Gets the output orientation, expressed in quaternions 
		* 
		* @return quaternion_out
		*/
		virtual eeros::control::Output<eeros::math::Vector4>& getOut_quaternion(){
			return quaternion_out;
		}
		
        /**
		* Gets the output linear accelerations ax, ay, az from accelerometer 
		* 
		* @return acc_out
		*/
		virtual eeros::control::Output<eeros::math::Vector3>& getOut_acc() {
			return acc_out;
		}
		
        /**
		* Gets the output angular velocities wx, wy, wz from gyro 
		* 
		* @return gyro_out
		*/
		virtual eeros::control::Output<eeros::math::Vector3>& getOut_gyro() {
			return gyro_out;
		}
		
        /**
		* Gets the output timestamp of the sensor signal 
		* 
		* @return timestamp_out
		*/
		virtual eeros::control::Output<uint32_t>& getOut_timestamp() {
			return timestamp_out;
		}
		
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

