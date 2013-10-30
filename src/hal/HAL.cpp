#include <eeros/hal/HAL.hpp>

using namespace eeros::hal;

HAL::HAL() { }
HAL::HAL(const HAL&) { }
HAL& HAL::operator=(const HAL&) { }

HAL& HAL::instance() {
	static HAL halInstance;
	return halInstance;
}

SystemOutput<bool>& HAL::getLogicSystemOutput(std::string name) {
	return *dynamic_cast<SystemOutput<bool>*>(outputs[name]);
}

SystemOutput<double>& HAL::getRealSystemOutput(std::string name) {
	return *dynamic_cast<SystemOutput<double>*>(outputs[name]);
}

SystemInput<bool>& HAL::getLogicSystemInput(std::string name) {
	return *dynamic_cast<SystemInput<bool>*>(outputs[name]);
}

SystemInput<double>& HAL::getRealSystemInput(std::string name) {
	return *dynamic_cast<SystemInput<double>*>(outputs[name]);
}