#include <eeros/control/BlockOutput.hpp>

using namespace eeros::control;

BlockOutput::BlockOutput(std::string id) : hal(hal::HAL::instance()) {
	systemOutput = hal.getRealSystemOutput(id);
	if(systemOutput == nullptr) throw -999;
}

void BlockOutput::run() {
	systemOutput->set(in.getValue());
}