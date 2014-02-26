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

SystemOutputInterface* HAL::getSystemOutput(std::string name, bool exclusive) {
	return outputs[name];
}

SystemOutput<bool>* HAL::getLogicSystemOutput(std::string name, bool exclusive) {
	SystemOutput<bool>* out = dynamic_cast<SystemOutput<bool>*>(outputs[name]);
	if(out == nullptr) throw EEROSException("Logic system output '" + name + "' not found!");
	if(exclusive) {
		if(!exclusiveReservedOutputs.insert(out).second) throw EEROSException("Logic system output '" + name + "' is exclusive reserved!");
	}
	return out;
}

SystemOutput<double>* HAL::getRealSystemOutput(std::string name, bool exclusive) {
	SystemOutput<double>* out = dynamic_cast<SystemOutput<double>*>(outputs[name]);
	if(out == nullptr) throw EEROSException("Real system output '" + name + "' not found!");
	if(exclusive) {
		if(!exclusiveReservedOutputs.insert(out).second) throw EEROSException("Real system output '" + name + "' is exclusive reserved!");
	}
	return out;
}

SystemInputInterface* HAL::getSystemInput(std::string name, bool exclusive) {
	return inputs[name];
}

SystemInput<bool>* HAL::getLogicSystemInput(std::string name, bool exclusive) {
	SystemInput<bool>* in = dynamic_cast<SystemInput<bool>*>(inputs[name]);
	if(in == nullptr) throw EEROSException("Logic system input '" + name + "' not found!");
	if(exclusive) {
		if(!exclusiveReservedInputs.insert(in).second) throw EEROSException("Logic system input '" + name + "' is exclusive reserved!");
	}
	return in;
}

SystemInput<double>* HAL::getRealSystemInput(std::string name, bool exclusive) {
	SystemInput<double>* in = dynamic_cast<SystemInput<double>*>(inputs[name]);
	if(in == nullptr) throw EEROSException("Real system input '" + name + "' not found!");
	if(exclusive) {
		if(!exclusiveReservedInputs.insert(in).second) throw EEROSException("Real system input '" + name + "' is exclusive reserved!");
	}
	return in;
}