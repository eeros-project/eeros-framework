#include <eeros/control/SpaceNavigatorInput.hpp>

using namespace eeros::control;
using namespace eeros::math;

SpaceNavigatorInput::SpaceNavigatorInput(std::string dev) {
	setInitPos({0,0,0});
	sn.open(dev.c_str());
	t = new std::thread([this](){ this->sn.loop(); });
}

SpaceNavigatorInput::~SpaceNavigatorInput() {
	delete t;
	sn.close();
}

void SpaceNavigatorInput::run() {
	out.getSignal().setValue(Matrix<SPACENAVIGATOR_AXIS_COUNT>{
		sn.current.axis[SpaceNav::Axis::X],
		sn.current.axis[SpaceNav::Axis::Y],
		sn.current.axis[SpaceNav::Axis::Z],
	});	
	uint64_t ts = eeros::System::getTimeNs();
	out.getSignal().setTimestamp(ts);
	rotOut.getSignal().setValue(Matrix<SPACENAVIGATOR_ROT_AXIS_COUNT>{
		sn.current.rotAxis[SpaceNav::RotAxis::RX],
		sn.current.rotAxis[SpaceNav::RotAxis::RY],
		sn.current.rotAxis[SpaceNav::RotAxis::RZ],
	});	
	buttonOut.getSignal().setValue(Matrix<SPACENAVIGATOR_BUTTON_COUNT,1,bool>{
		sn.current.button[SpaceNav::Button::L],
		sn.current.button[SpaceNav::Button::R]
	});
	buttonOut.getSignal().setTimestamp(ts);
}

Output<Matrix<SPACENAVIGATOR_ROT_AXIS_COUNT,1,double>>& SpaceNavigatorInput::getRotOut() {
	return rotOut;
}

Output<Matrix<SPACENAVIGATOR_BUTTON_COUNT,1,bool>>& SpaceNavigatorInput::getButtonOut() {
	return buttonOut;
}

void SpaceNavigatorInput::setInitPos(Matrix<SPACENAVIGATOR_AXIS_COUNT> initPos) {
	out.getSignal().setValue(initPos);
}


