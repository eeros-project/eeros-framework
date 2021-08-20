#include <iostream>
#include <eeros/control/sensors/BaumerOM70Input.hpp>

using namespace eeros::control;
using namespace eeros::math;


BaumerOM70Input::BaumerOM70Input(std::string dev, int port, int slave_id, int priority) : 
om70(dev, port, slave_id, priority),
log(Logger::getLogger()) 
{ }

void BaumerOM70Input::run() {
    this->out.getSignal().setValue(om70.get_distance());
    this->out.getSignal().setTimestamp(eeros::System::getTimeNs());
}

