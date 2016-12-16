#include <eeros/hal/HAL.hpp>
#include <eeros/core/EEROSException.hpp>
#include <dlfcn.h>

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
	
	parser = JsonParser(file);
	parser.createHalObjects(hwLibraries);
	// TODO parse file, load necessary modules and delegate creation of the system in- and output objects
	return false;
}

bool HAL::loadModule(std::string moduleName) {
	// TODO
	return false;
}

bool HAL::addInput(InputInterface* systemInput) {
	if(systemInput != nullptr) {
		inputs.insert(std::pair<std::string, InputInterface*>(systemInput->getId(), systemInput));
		return true;
	}
	return false;
}
bool HAL::addOutput(OutputInterface* systemOutput) {
	if(systemOutput != nullptr) {
		outputs.insert(std::pair<std::string, OutputInterface*>(systemOutput->getId(), systemOutput));
		return true;
	}
	return false;
}

OutputInterface* HAL::getOutput(std::string name, bool exclusive) {
	return outputs[name];
}

Output<bool>* HAL::getLogicOutput(std::string name, bool exclusive) {
	Output<bool>* out = dynamic_cast<Output<bool>*>(outputs[name]);
	if(out == nullptr) throw EEROSException("Logic system output '" + name + "' not found!");
	if(exclusive) {
		if(!exclusiveReservedOutputs.insert(out).second) throw EEROSException("Logic system output '" + name + "' is exclusive reserved!");
	}
	return out;
}

Output<double>* HAL::getRealOutput(std::string name, bool exclusive) {
	Output<double>* out = dynamic_cast<Output<double>*>(outputs[name]);
	if(out == nullptr) throw EEROSException("Real system output '" + name + "' not found!");
	if(exclusive) {
		if(!exclusiveReservedOutputs.insert(out).second) throw EEROSException("Real system output '" + name + "' is exclusive reserved!");
	}
	return out;
}

InputInterface* HAL::getInput(std::string name, bool exclusive) {
	return inputs[name];
}

Input<bool>* HAL::getLogicInput(std::string name, bool exclusive) {
	Input<bool>* in = dynamic_cast<Input<bool>*>(inputs[name]);
	if(in == nullptr) throw EEROSException("Logic system input '" + name + "' not found!");
	if(exclusive) {
		if(!exclusiveReservedInputs.insert(in).second) throw EEROSException("Logic system input '" + name + "' is exclusive reserved!");
	}
	return in;
}

Input<double>* HAL::getRealInput(std::string name, bool exclusive) {
	Input<double>* in = dynamic_cast<Input<double>*>(inputs[name]);
	if(in == nullptr) throw EEROSException("Real system input '" + name + "' not found!");
	if(exclusive) {
		if(!exclusiveReservedInputs.insert(in).second) throw EEROSException("Real system input '" + name + "' is exclusive reserved!");
	}
	return in;
}

void HAL::getOutputFeature(std::string name, std::string featureName, void *(handle)() ){
// 	auto outObj = outputs[name];
// // 	d
// 	handle = reinterpret_cast<void*()>(dlsym(outObj->getLibHandle(), featureName.c_str()));
// 	if(handle == nullptr){
// 		throw new eeros::EEROSException("could not find method in dynamic library");
// 		std::cout << "err: " << dlerror() << std::endl;
// 	}
}
