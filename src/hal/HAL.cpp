#include <eeros/hal/HAL.hpp>
#include <eeros/core/Fault.hpp>

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

bool HAL::addPeripheralInput(PeripheralInputInterface* systemInput) {
	if(systemInput != nullptr) {
		inputs.insert(std::pair<std::string, PeripheralInputInterface*>(systemInput->getId(), systemInput));
		return true;
	}
	return false;
}
bool HAL::addPeripheralOutput(PeripheralOutputInterface* systemOutput) {
	if(systemOutput != nullptr) {
		outputs.insert(std::pair<std::string, PeripheralOutputInterface*>(systemOutput->getId(), systemOutput));
		return true;
	}
	return false;
}

PeripheralOutputInterface* HAL::getPeripheralOutput(std::string name, bool exclusive) {
	return outputs[name];
}

PeripheralOutput<bool>* HAL::getLogicPeripheralOutput(std::string name, bool exclusive) {
	PeripheralOutput<bool>* out = dynamic_cast<PeripheralOutput<bool>*>(outputs[name]);
	if(out == nullptr) throw Fault("Logic system output '" + name + "' not found!");
	if(exclusive) {
		if(!exclusiveReservedOutputs.insert(out).second) throw Fault("Logic system output '" + name + "' is exclusive reserved!");
	}
	return out;
}

PeripheralOutput<double>* HAL::getRealPeripheralOutput(std::string name, bool exclusive) {
	PeripheralOutput<double>* out = dynamic_cast<PeripheralOutput<double>*>(outputs[name]);
	if(out == nullptr) throw Fault("Real system output '" + name + "' not found!");
	if(exclusive) {
		if(!exclusiveReservedOutputs.insert(out).second) throw Fault("Real system output '" + name + "' is exclusive reserved!");
	}
	return out;
}

PeripheralInputInterface* HAL::getPeripheralInput(std::string name, bool exclusive) {
	return inputs[name];
}

PeripheralInput<bool>* HAL::getLogicPeripheralInput(std::string name, bool exclusive) {
	PeripheralInput<bool>* in = dynamic_cast<PeripheralInput<bool>*>(inputs[name]);
	if(in == nullptr) throw Fault("Logic system input '" + name + "' not found!");
	if(exclusive) {
		if(!exclusiveReservedInputs.insert(in).second) throw Fault("Logic system input '" + name + "' is exclusive reserved!");
	}
	return in;
}

PeripheralInput<double>* HAL::getRealPeripheralInput(std::string name, bool exclusive) {
	PeripheralInput<double>* in = dynamic_cast<PeripheralInput<double>*>(inputs[name]);
	if(in == nullptr) throw Fault("Real system input '" + name + "' not found!");
	if(exclusive) {
		if(!exclusiveReservedInputs.insert(in).second) throw Fault("Real system input '" + name + "' is exclusive reserved!");
	}
	return in;
}
