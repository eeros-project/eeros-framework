#include <iostream>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/core/System.hpp>
#include "ODriveInput_UART.hpp"

using namespace eeros::control;
using namespace eeros::math;
using namespace slingload;


ODriveInput_UART::ODriveInput_UART(std::string dev, int speed, int parity, int priority):
odrive(dev, speed, parity, priority) {}

ODriveInput_UART::~ODriveInput_UART() { }

void ODriveInput_UART::run() {
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

eeros::control::Output<Vector2>& ODriveInput_UART::getVelOut() {
    return velOut;
}


