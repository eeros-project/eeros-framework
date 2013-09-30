#include <eeros/hal/HAL.hpp>

HAL::HAL() { }
HAL::HAL(const HAL&) { }
HAL& HAL::operator=(const HAL&) { }

HAL& HAL::instance() {
	static HAL halInstance;
	return halInstance;
}

SystemOutput<bool> HAL::getLogicSystemOutput(std::string name) {
	// TODO
	return SystemOutput<bool>();
}

SystemOutput<double> HAL::getRealSystemOutput(std::string name) {
	// TODO
	return SystemOutput<double>();
}

SystemInput<bool> HAL::getLogicSystemInput(std::string name) {
	// TODO
	return SystemInput<bool>();
}

SystemInput<double> HAL::getRealSystemInput(std::string name) {
	// TODO
	return SystemInput<double>();
}