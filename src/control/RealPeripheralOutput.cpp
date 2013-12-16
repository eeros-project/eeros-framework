#include <eeros/control/RealPeripheralOutput.hpp>

using namespace eeros::control;

RealPeripheralOutput::RealPeripheralOutput(std::string outputId, double scale, double offset) : hal(hal::HAL::instance()), scale(scale), offset(offset) {
	systemOutput = hal.getRealSystemOutput(outputId);
	if(systemOutput == nullptr) throw -999;
}

void RealPeripheralOutput::setOffset(double o) {
	offset = o;
}

void RealPeripheralOutput::setScale(double s) {
	scale = s;
}

void RealPeripheralOutput::run() {
	systemOutput->set(in.getValue() * scale + offset);
}