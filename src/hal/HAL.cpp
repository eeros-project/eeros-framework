#include <eeros/hal/HAL.hpp>
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
	if( exclusiveReservedOutputs.find(outputs[name]) != exclusiveReservedOutputs.end() ) throw EEROSException("System output '" + name + "' is exclusive reserved!");
	
	if(exclusive) {
		if( nonExclusiveOutputs.find(outputs[name]) != nonExclusiveOutputs.end() ){
			throw EEROSException("System output '" + name + "' is already claimed as non-exclusive output!");
		}
		if(!exclusiveReservedOutputs.insert(outputs[name]).second) throw EEROSException("System output '" + name + "' is exclusive reserved!"); // should not fail here because already checked at the beginning
	}
	else{
		nonExclusiveOutputs.insert(outputs[name]).second;
	}
	return outputs[name];
}

Output<bool>* HAL::getLogicOutput(std::string name, bool exclusive) {
	Output<bool>* out = dynamic_cast<Output<bool>*>(outputs[name]);
	if(out == nullptr) throw EEROSException("Logic system output '" + name + "' not found!");
	
	if( exclusiveReservedOutputs.find(outputs[name]) != exclusiveReservedOutputs.end() ) throw EEROSException("Logic system output '" + name + "' is exclusive reserved!");
	
	if(exclusive) {
		if( nonExclusiveOutputs.find(outputs[name]) != nonExclusiveOutputs.end() ){
			throw EEROSException("Logic system output '" + name + "' is already claimed as non-exclusive output!");
		}
		if(!exclusiveReservedOutputs.insert(outputs[name]).second) throw EEROSException("Logic system output '" + name + "' is exclusive reserved!"); // should not fail here because already checked at the beginning
	}
	else{
		nonExclusiveOutputs.insert(outputs[name]).second;
	}
	return out;
}

Output<double>* HAL::getRealOutput(std::string name, bool exclusive) {
	Output<double>* out = dynamic_cast<Output<double>*>(outputs[name]);
	if(out == nullptr) throw EEROSException("Real system output '" + name + "' not found!");
	
	if( exclusiveReservedOutputs.find(outputs[name]) != exclusiveReservedOutputs.end() ) throw EEROSException("Real system output '" + name + "' is exclusive reserved!");
	
	if(exclusive) {
		if( nonExclusiveOutputs.find(outputs[name]) != nonExclusiveOutputs.end() ){
			throw EEROSException("Real system output '" + name + "' is already claimed as non-exclusive output!");
		}
		if(!exclusiveReservedOutputs.insert(outputs[name]).second) throw EEROSException("Real system output '" + name + "' is exclusive reserved!"); // should not fail here because already checked at the beginning
	}
	else{
		nonExclusiveOutputs.insert(outputs[name]).second;
	}
	return out;
}

InputInterface* HAL::getInput(std::string name, bool exclusive) {
	if( exclusiveReservedInputs.find(inputs[name]) != exclusiveReservedInputs.end() ) throw EEROSException("System input '" + name + "' is exclusive reserved!");
	
	if(exclusive) {
		if( nonExclusiveInputs.find(inputs[name]) != nonExclusiveInputs.end() ){
			throw EEROSException("System input '" + name + "' is already claimed as non-exclusive input!");
		}
		if(!exclusiveReservedInputs.insert(inputs[name]).second) throw EEROSException("System input '" + name + "' is exclusive reserved!");	// should not fail here because already checked at the beginning
	}
	else{	
		nonExclusiveInputs.insert(inputs[name]).second;
	}
	return inputs[name];
}

Input<bool>* HAL::getLogicInput(std::string name, bool exclusive) {
	Input<bool>* in = dynamic_cast<Input<bool>*>(inputs[name]);
	if(in == nullptr) throw EEROSException("Logic system input '" + name + "' not found!");
	
	if( exclusiveReservedInputs.find(inputs[name]) != exclusiveReservedInputs.end() ) throw EEROSException("Logic system input '" + name + "' is exclusive reserved!");
		
	if(exclusive) {
		if( nonExclusiveInputs.find(inputs[name]) != nonExclusiveInputs.end() ){
			throw EEROSException("Logic system input '" + name + "' is already claimed as non-exclusive input!");
		}
		if(!exclusiveReservedInputs.insert(inputs[name]).second) throw EEROSException("Logic system input '" + name + "' is exclusive reserved!");	// should not fail here because already checked at the beginning
	}
	else{	
		nonExclusiveInputs.insert(inputs[name]).second;
	}
	return in;
}

Input<double>* HAL::getRealInput(std::string name, bool exclusive) {
	Input<double>* in = dynamic_cast<Input<double>*>(inputs[name]);
	if(in == nullptr) throw EEROSException("Real system input '" + name + "' not found!");
	
	if( exclusiveReservedInputs.find(inputs[name]) != exclusiveReservedInputs.end() ) throw EEROSException("Real system input '" + name + "' is exclusive reserved!");
	
	if(exclusive) {
		if( nonExclusiveInputs.find(inputs[name]) != nonExclusiveInputs.end() ){
			throw EEROSException("Real system input '" + name + "' is already claimed as non-exclusive input!");
		}
		if(!exclusiveReservedInputs.insert(inputs[name]).second) throw EEROSException("Real system input '" + name + "' is exclusive reserved!");	// should not fail here because already checked at the beginning
	}
	else{	
		nonExclusiveInputs.insert(inputs[name]).second;
	}
	return in;
}

void * HAL::getOutputFeature(std::string name, std::string featureName){
	auto outObj = outputs[name];
	return getOutputFeature(outObj, featureName);
}

void* HAL::getOutputFeature(eeros::hal::OutputInterface * obj, std::string featureName){
	return dlsym(obj->getLibHandle(), featureName.c_str());
}
