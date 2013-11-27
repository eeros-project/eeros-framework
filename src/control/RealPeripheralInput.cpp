#include <eeros/control/RealPeripheralInput.hpp>
#include <eeros/core/System.hpp>

using namespace eeros;
using namespace eeros::control;

RealPeripheralInput::RealPeripheralInput(std::string outputId) : hal(hal::HAL::instance()), Block1o(1) {
	systemInput = hal.getRealSystemInput(outputId);
	if(systemInput == nullptr) throw -999;
}

void RealPeripheralInput::run() {
	out.setValue(systemInput->get());
	out.setTimeStamp(System::getTimeNs());
}