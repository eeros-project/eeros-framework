#include <iostream>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/control/sensors/SBGEllipseAInput.hpp>

using namespace eeros::math;
using namespace eeros::control;

SBGEllipseAInput::SBGEllipseAInput(std::string dev, int priority) : 
sbg(dev, priority),
log(Logger::getLogger()) 
{}

void SBGEllipseAInput::run() {
	euler_out.getSignal().setValue(SBGEllipseA::data_euler);
    quaternion_out.getSignal().setValue(SBGEllipseA::data_quat);
    acc_out.getSignal().setValue(SBGEllipseA::data_acc);
    gyro_out.getSignal().setValue(SBGEllipseA::data_gyro);
	timestamp_out.getSignal().setValue(SBGEllipseA::timestamp_euler);
        
    // Timestamps
    uint64_t ts = eeros::System::getTimeNs();
    euler_out.getSignal().setTimestamp(ts);
	quaternion_out.getSignal().setTimestamp(ts);
	acc_out.getSignal().setTimestamp(ts);
	gyro_out.getSignal().setTimestamp(ts);
	timestamp_out.getSignal().setTimestamp(ts);
}

eeros::control::Output<Vector3>& SBGEllipseAInput::getOut_eulerAng() {
    return euler_out;
}

eeros::control::Output<Vector4>& SBGEllipseAInput::getOut_quaternion() {
    return quaternion_out;
}

eeros::control::Output<Vector3>& SBGEllipseAInput::getOut_acc() {
    return acc_out;
}

eeros::control::Output<Vector3>& SBGEllipseAInput::getOut_gyro() {
    return gyro_out;
}

eeros::control::Output<uint32_t>& SBGEllipseAInput::getOut_timestamp() {
    return timestamp_out;
}

