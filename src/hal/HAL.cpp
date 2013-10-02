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
	static bool value = false;
	return SystemOutput<bool>(value);
}

SystemOutput<double> HAL::getRealSystemOutput(std::string name) {
	// TODO
	static double value = 0;
	return SystemOutput<double>(value);
}

SystemInput<bool> HAL::getLogicSystemInput(std::string name) {
	// TODO
	static bool value = false;
	return SystemInput<bool>(value);
}

SystemInput<double> HAL::getRealSystemInput(std::string name) {
	// TODO
	static double value = 0;
	return SystemInput<double>(value);
}