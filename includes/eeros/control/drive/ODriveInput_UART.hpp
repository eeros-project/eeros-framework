#ifndef ORG_EEROS_CONTROL_ODRIVEINPUT_UART_HPP
#define ORG_EEROS_CONTROL_ODRIVEINPUT_UART_HPP

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/hal/Odrive_UART.hpp>
#include <eeros/core/PeriodicCounter.hpp>


namespace eeros {
namespace control {
	class ODriveInput_UART: public eeros::control::Blockio<0,1,eeros::math::Vector2> {

	public:
		/**
		* Constructs a ODriveInput_USB instance \n
		* @param dev - string with device id
		* @param speed - communication speed of UART
		* @param parity - parity bit set or not
		* @param priority - execution priority of this thread
		*/
		ODriveInput_UART(std::string dev, int speed, int parity, int priority):
		odrive(dev, speed, parity, priority) {}
		
		/**
		* Disabling use of copy constructor because the block should never be copied unintentionally.
		*/
		ODriveInput_UART(const ODriveInput_UART& s) = delete; 
		
		/**
		* Gets input data from ODrive_USB Thread and outputs them
		*/
		virtual void run() {
			// 	counter.tick();
				
			// Output data 
			double vel0 = odrive.get_actual_vel(0);
			double vel1 = odrive.get_actual_vel(1);
			Vector2 vel; vel << vel0, vel1;
			
			velOut.getSignal().setValue(vel);
					
			// Timestamps 
			uint64_t ts = eeros::System::getTimeNs();
			
			// 	counter.tock();
			// 	
			// 	static int count = 0;
			//         if(count % 1000 == 0){
			// 			std::cout << "run mean: " << counter.run.mean << ", period mean: " << counter.period.mean << ", period max: " << counter.period.max << std::endl;
			// 		}
			//         count++;
		}
		
		/**
		* Returns velocity to be set on odrives
		*/
		virtual eeros::control::Output<eeros::math::Vector2>& getVelOut();

	private:
		std::thread* t;
		ODrive_UART odrive;

	protected:
		eeros::PeriodicCounter counter;
		eeros::control::Output<eeros::math::Vector2> velOut {
			return velOut;
		}
	};
}
}

#endif /* ORG_EEROS_CONTROL_ODRIVEINPUT_UART_HPP */
