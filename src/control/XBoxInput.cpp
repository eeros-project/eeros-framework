#include <eeros/control/XBoxInput.hpp>

using namespace eeros::control;
using namespace eeros::math;

XBoxInput::XBoxInput(std::string dev) : x(dev) {
	setInitPos({0,0,0,0,0,0,0,0});
	axisScale << xScale,      0,      0,      0,
	                  0, yScale,      0,      0,
	                  0,      0, zScale,      0,
	                  0,      0,      0, rScale;
}

XBoxInput::~XBoxInput() { }

void XBoxInput::run() {
	out.getSignal().setValue(Matrix<XBOX_AXIS_COUNT>{
		x.current.axis[XBoxController::Axis::LX],
		x.current.axis[XBoxController::Axis::LY],
		x.current.axis[XBoxController::Axis::LT],
		x.current.axis[XBoxController::Axis::RX],
		x.current.axis[XBoxController::Axis::RY],
		x.current.axis[XBoxController::Axis::RT],
		x.current.axis[XBoxController::Axis::CX],
		x.current.axis[XBoxController::Axis::CY]
	});	
	uint64_t ts = eeros::System::getTimeNs();
	out.getSignal().setTimestamp(ts);
	buttonOut.getSignal().setValue(Matrix<XBOX_BUTTON_COUNT,1,bool>{
		x.current.button_state[XBoxController::Button::A],
		x.current.button_state[XBoxController::Button::B],
		x.current.button_state[XBoxController::Button::X],
		x.current.button_state[XBoxController::Button::Y],
		x.current.button_state[XBoxController::Button::LB],
		x.current.button_state[XBoxController::Button::RB],
		x.current.button_state[XBoxController::Button::back],
		x.current.button_state[XBoxController::Button::start]
	});
	buttonOut.getSignal().setTimestamp(ts);
}

Output<Matrix<XBOX_BUTTON_COUNT,1,bool>>& XBoxInput::getButtonOut() {
	return buttonOut;
}

void XBoxInput::setInitPos(Matrix<XBOX_AXIS_COUNT> initPos) {
	out.getSignal().setValue(initPos);
}

void XBoxInput::setSpeedScaleFactor(double speedScale) {
	if(speedScale >= 0 && speedScale < 5)
		speedScaleFactor = speedScale;
}

