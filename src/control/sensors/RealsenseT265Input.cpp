#include <iostream>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/sensors/RealsenseT265Input.hpp>

using namespace eeros::control;
using namespace eeros::math;


RealsenseT265Input::RealsenseT265Input(std::string dev, int priority) : 
t265(dev, priority) {}

void RealsenseT265Input::run() {
    // Get data from camera
    out_translation.getSignal().setValue(t265.translation);
    out_velocity.getSignal().setValue(t265.velocity);
    out_acceleration.getSignal().setValue(t265.acceleration);
    out_angular_velocity.getSignal().setValue(t265.angular_velocity);
    out_angular_acceleration.getSignal().setValue(t265.angular_acceleration);
    out_quaternion.getSignal().setValue(t265.quaternion);
        
    // Timestamps
    uint64_t ts = eeros::System::getTimeNs();
	out_translation.getSignal().setTimestamp(ts);
	out_velocity.getSignal().setTimestamp(ts);
	out_acceleration.getSignal().setTimestamp(ts);
	out_angular_velocity.getSignal().setTimestamp(ts);
	out_angular_acceleration.getSignal().setTimestamp(ts);
	out_quaternion.getSignal().setTimestamp(ts);
}


Output<Vector3>& RealsenseT265Input::getOut_translation(){
    return out_translation;
}

Output<Vector3>& RealsenseT265Input::getOut_velocity(){
    return out_velocity;
}

Output<Vector3>& RealsenseT265Input::getOut_acceleration(){
    return out_acceleration;
}

Output<Vector3>& RealsenseT265Input::getOut_angularVel(){
    return out_angular_velocity;
}

Output<Vector3>& RealsenseT265Input::getOut_angularAcc(){
    return out_angular_acceleration;
}

Output<Vector4>& RealsenseT265Input::getOut_quaternion(){
    return out_quaternion;
}



