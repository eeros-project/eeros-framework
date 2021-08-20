#include <iostream>
// #include <eeros/math/Matrix.hpp>
#include <eeros/control/sensors/RPLidarInput.hpp>

using namespace eeros::control;
using namespace eeros::math;


RPLidarInput::RPLidarInput(std::string dev, int priority) : 
rplidar(dev, priority) {}

void RPLidarInput::run()
{
	// Set output data
	this->out[0].getSignal().setValue(rplidar.get_angles());
	this->out[1].getSignal().setValue(rplidar.get_ranges());
	
	// Set output timestamp
    uint64_t ts = eeros::System::getTimeNs();
	this->out.getSignal().setTimestamp(ts);
}

