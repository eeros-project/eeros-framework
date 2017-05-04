#include <eeros/control/KeyboardInput.hpp>

using namespace eeros::control;

KeyboardInput::KeyboardInput() {
	t = new std::thread([this](){ this->k.loop(); });
}

KeyboardInput::~KeyboardInput() {
	delete t;
}

void KeyboardInput::run() {
	uint64_t time = eeros::System::getTimeNs();
	out.getSignal().setTimestamp(time);
	out.getSignal().setValue(Vector4{k.speed[0], k.speed[1], k.speed[2], k.speed[3]});
	
	isHomed.getSignal().setTimestamp(time);
	isHomed.getSignal().setValue(Vector<5,bool>{k.homed[0], k.homed[1], k.homed[2], k.homed[3], k.homed[4]});
	
	esc.getSignal().setValue(k.events.esc);
	esc.getSignal().setTimestamp(time);
	emergency.getSignal().setValue(k.events.emergency);
	emergency.getSignal().setTimestamp(time);
	reset.getSignal().setValue(k.events.reset);
	reset.getSignal().setTimestamp(time);
	start.getSignal().setValue(k.events.start);
	start.getSignal().setTimestamp(time);
	stop.getSignal().setValue(k.events.stop);
	stop.getSignal().setTimestamp(time);
}

Output<Vector<5,bool>>& KeyboardInput::getIsHomed() {
	return isHomed;
}

Output<bool>& KeyboardInput::getEsc() {
	return esc;
}

Output<bool>& KeyboardInput::getEmergency() {
	return emergency;
}

Output<bool>& KeyboardInput::getReset() {
	return reset;
}

Output<bool>& KeyboardInput::getStart() {
	return start;
}

Output<bool>& KeyboardInput::getStop() {
	return stop;
}
