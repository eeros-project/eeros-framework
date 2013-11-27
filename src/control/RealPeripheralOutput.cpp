#include <eeros/control/RealPeripheralOutput.hpp>

using namespace eeros::control;

RealPeripheralOutput::RealPeripheralOutput(std::string outputId) : hal(hal::HAL::instance()) {
	systemOutput = hal.getRealSystemOutput(outputId);
	if(systemOutput == nullptr) throw -999;
}

void RealPeripheralOutput::run() {
	systemOutput->set(in.getValue());
}