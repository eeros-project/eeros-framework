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
	return true;
}

bool HAL::loadModule(std::string moduleName) {
	// TODO
	return false;
}

bool HAL::addInput(InputInterface* systemInput) {
	if(systemInput != nullptr) {
		if( inputs.find(systemInput->getId()) != inputs.end() ){
			throw EEROSException("Could not add Input to HAL, signal id '" + systemInput->getId() + "' already exists!");
		}
		inputs.insert(std::pair<std::string, InputInterface*>(systemInput->getId(), systemInput));
		return true;
	}
	throw EEROSException("System input is null");
}
bool HAL::addOutput(OutputInterface* systemOutput) {
	if(systemOutput != nullptr) {
		if( outputs.find(systemOutput->getId()) != outputs.end() ){
			throw EEROSException("Could not add Output to HAL, signal id '" + systemOutput->getId() + "' already exists!");
		}
		outputs.insert(std::pair<std::string, OutputInterface*>(systemOutput->getId(), systemOutput));
		return true;
	}
	throw EEROSException("System output is null");
}

void HAL::releaseInput(std::string name) {
	bool found = false;
	auto inIt = nonExclusiveInputs.find(inputs[name]);
	if(inIt != nonExclusiveInputs.end()){
		nonExclusiveInputs.erase(inIt);
		found = true;
	}
	  
	inIt = exclusiveReservedInputs.find(inputs[name]);
	if(inIt != exclusiveReservedInputs.end()){
		exclusiveReservedInputs.erase(inIt);
		found = true;
	}
	if(!found){
		throw EEROSException("Could not release system input '" + name + "', id not found.");
	}
}

void HAL::releaseOutput(std::string name) {
	bool found = false;
	auto outIt = nonExclusiveOutputs.find(outputs[name]);
	if(outIt != nonExclusiveOutputs.end()){
		nonExclusiveOutputs.erase(outIt);
		found = true;
	}
	
	outIt = exclusiveReservedOutputs.find(outputs[name]);
	if(outIt != exclusiveReservedOutputs.end()){
		exclusiveReservedOutputs.erase(outIt);
		found = true;
	}
	if(!found){
		throw EEROSException("Could not release system output '" + name + "', id not found.");
	}
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

ScalableOutput<double>* HAL::getRealOutput(std::string name, bool exclusive) {
	ScalableOutput<double>* out = dynamic_cast<ScalableOutput<double>*>(outputs[name]);
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

ScalableInput<double>* HAL::getRealInput(std::string name, bool exclusive) {
	ScalableInput<double>* in = dynamic_cast<ScalableInput<double>*>(inputs[name]);
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

void* HAL::getOutputFeature(OutputInterface * obj, std::string featureName){
	return dlsym(obj->getLibHandle(), featureName.c_str());
}

void * HAL::getInputFeature(std::string name, std::string featureName){
	auto inObj = inputs[name];
	return getInputFeature(inObj, featureName);
}

void* HAL::getInputFeature(InputInterface * obj, std::string featureName){
	return dlsym(obj->getLibHandle(), featureName.c_str());
}
