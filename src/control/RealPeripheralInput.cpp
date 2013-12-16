#include <eeros/control/RealPeripheralInput.hpp>
#include <eeros/core/System.hpp>

using namespace eeros;
using namespace eeros::control;

RealPeripheralInput::RealPeripheralInput(std::string outputId, double scale, double offset) : hal(hal::HAL::instance()), scale(scale), offset(offset), Block1o(1) {
	systemInput = hal.getRealSystemInput(outputId);
	if(systemInput == nullptr) throw -999;
}

void RealPeripheralInput::setOffset(double o) {
	offset = o;
}

void RealPeripheralInput::setScale(double s) {
	scale = s;
}

void RealPeripheralInput::run() {
	out.setValue(systemInput->get() * scale + offset);
	out.setTimeStamp(System::getTimeNs());
}