#ifndef ORG_EEROS_CONTROL_ODRIVEINPUT_USB_HPP
#define ORG_EEROS_CONTROL_ODRIVEINPUT_USB_HPP

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/hal/ODrive_USB.hpp>
#include <eeros/core/PeriodicCounter.hpp>


namespace eeros {
namespace control {
	class ODriveInput_USB: public eeros::control::Blockio<1,3,eeros::math::Vector2> {

	public:
		/**
		* Constructs a ODriveInput_USB instance \n
		* @param odrive_serialnr - device serial number
		* @param enc_ticks - encoder ticks of motors connected
		* @param priority - execution priority of this thread
		* @param first_drive - one drive must be set as the first one for reset tasks 
		*/
		ODriveInput_USB(uint64_t odrive_serialnr, float enc_ticks, int priority, bool first_drive) :
		odrive(odrive_serialnr, enc_ticks, priority, first_drive) {}
		
		/**
		* Disabling use of copy constructor because the block should never be copied unintentionally.
		*/
		ODriveInput_USB(const ODriveInput_USB& s) = delete; 
		
		/**
		* Gets input data from ODrive_USB Thread and outputs them
		*/
		virtual void run(){
			// 	counter.tick();
				
			// Set speed
			odrive.set_ref_vel(0, velRefIn.getSignal().getValue()(0)); // rad/s
			odrive.set_ref_vel(1, velRefIn.getSignal().getValue()(1)); // rad/s
			
			// Output data Value
			double vel0 = odrive.get_encoder_vel(0); // rad/s
			double vel1 = odrive.get_encoder_vel(1); // rad/s
			Vector2 vel; vel << vel0, vel1;
			velActOut.getSignal().setValue(vel);
			
			// Current
			double iq_meas_0 = odrive.get_current_measured(0);
			double iq_meas_1 = odrive.get_current_measured(1);
			Vector2 iq_meas; iq_meas << iq_meas_0, iq_meas_1;
			currMeasOut.getSignal().setValue(iq_meas);
			
			double iq_setp_0 = odrive.get_current_setpoint(0);
			double iq_setp_1 = odrive.get_current_setpoint(1);
			Vector2 iq_setp; iq_meas << iq_setp_0, iq_setp_1;
			currSetpOut.getSignal().setValue(iq_setp);
					
			// Output data Timestamp
			uint64_t ts = eeros::System::getTimeNs();
			velActOut.getSignal().setTimestamp(ts);
			currMeasOut.getSignal().setTimestamp(ts);
			currSetpOut.getSignal().setTimestamp(ts);
			
			// 	counter.tock();
				
			// 	//Periodic counter test
			// 	static int count = 0;
			//         if(count % 1000 == 0){
			// 			std::cout << "run mean: " << counter.run.mean << ", period mean: " << counter.period.mean << ", period max: " << counter.period.max << std::endl;
			// 		}
			// 	count++;
		}
		
		/**
		* Returns input velocity, determined from control system, to be set on odrives
		*/
		virtual eeros::control::Input<eeros::math::Vector2>& getVelRefIn(){
			return velRefIn;
		}
		/**
		* Returns actual motor speed (from encoders)
		*/
		virtual eeros::control::Output<eeros::math::Vector2>& getVelActOut(){
			return velActOut;
		}
		/**
		* Returns actual current setpoint for odrives / motors
		*/
		virtual eeros::control::Output<eeros::math::Vector2>& getCurrSetpOut(){
			return currSetpOut;
		}
		/**
		* Returns actual current measured on odrives / motors
		*/
		virtual eeros::control::Output<eeros::math::Vector2>& getCurrMeasOut(){
			return currMeasOut;
		}
		
		/**
		* Enables drives
		*/
		virtual void enable() {
			odrive.enable_drives();
		}
		/**
		* Disables drives
		*/
		virtual void disable() {
			odrive.disable_drives();
		}
		
		/**
		* Disables drives
		*/
		virtual bool ODriveInput_USB::is_odrive_emergency() {
			return odrive.is_endstop_active();
		}
		/**
		* Returns true, if odrive is calibrated
		*/
		virtual bool ODriveInput_USB::is_odrive_calibrated() {
			return odrive.is_calibrated();
		}
		/**
		* Starts odrive calibration procedure
		*/
		virtual void ODriveInput_USB::do_odrive_calibration() {
			odrive.do_start_calibration();
		}
		
	private:
		std::thread* t;
		ODrive_USB odrive;

	protected:
		eeros::PeriodicCounter counter;
		eeros::control::Input<eeros::math::Vector2> velRefIn;
		eeros::control::Output<eeros::math::Vector2> velActOut;
		eeros::control::Output<eeros::math::Vector2> currSetpOut;
		eeros::control::Output<eeros::math::Vector2> currMeasOut;
	};
}
}

#endif /* ORG_EEROS_CONTROL_ODRIVEINPUT_USB_HPP */
