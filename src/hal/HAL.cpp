#include <eeros/hal/HAL.hpp>
#include <eeros/core/EEROSException.hpp>

using namespace eeros;
using namespace eeros::hal;

HAL::HAL() { }
HAL::HAL(const HAL&) { }
HAL& HAL::operator=(const HAL&) { }

HAL& HAL::instance() {
	static HAL halInstance;
	return halInstance;
}

bool HAL::readConfigFromFile(std::string file) {
	// TODO parse file, load necessary modules and delegate creation of the system in- and output objects
	return false;
}

bool HAL::loadModule(std::string moduleName) {
	// TODO
	return false;
}

bool HAL::addSystemInput(SystemInputInterface* systemInput) {
	if(systemInput != nullptr) {
		inputs.insert(std::pair<std::string, SystemInputInterface*>(systemInput->getId(), systemInput));
		return true;
	}
	return false;
}
bool HAL::addSystemOutput(SystemOutputInterface* systemOutput) {
	if(systemOutput != nullptr) {
		outputs.insert(std::pair<std::string, SystemOutputInterface*>(systemOutput->getId(), systemOutput));
		return true;
	}
	return false;
}

SystemOutput<bool>& HAL::getLogicSystemOutput(std::string name) {
	SystemOutput<bool>* out = dynamic_cast<SystemOutput<bool>*>(outputs[name]);
	if(out == nullptr) throw EEROSException("Logic system output '" + name + "' not found!");
	return *out;
}

SystemOutput<double>& HAL::getRealSystemOutput(std::string name) {
	SystemOutput<double>* out = dynamic_cast<SystemOutput<double>*>(outputs[name]);
	if(out == nullptr) throw EEROSException("Real system output '" + name + "' not found!");
	return *out;
}

SystemInput<bool>& HAL::getLogicSystemInput(std::string name) {
	SystemInput<bool>* in = dynamic_cast<SystemInput<bool>*>(inputs[name]);
	if(in == nullptr) throw EEROSException("Logic system input '" + name + "' not found!");
	return *in;
}

SystemInput<double>& HAL::getRealSystemInput(std::string name) {
	SystemInput<double>* in = dynamic_cast<SystemInput<double>*>(inputs[name]);
	if(in == nullptr) throw EEROSException("Real system input '" + name + "' not found!");
	return *in;
}