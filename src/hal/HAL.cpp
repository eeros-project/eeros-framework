#include <eeros/hal/HAL.hpp>
#include <eeros/core/Fault.hpp>
#include <dlfcn.h>
#include <getopt.h>

using namespace eeros;
using namespace eeros::hal;

HAL::HAL() : log('H')  { }
HAL::HAL(const HAL&) : log('H') { }

HAL& HAL::instance() {
	static HAL halInstance;
	return halInstance;
}

bool HAL::readConfigFromFile(std::string file) {
	parser = JsonParser(file);
	parser.createHalObjects(hwLibraries);
	return true;
}

bool HAL::readConfigFromFile(int* argc, char** argv) {
	if (*argc < 3) throw Fault("no configuration file given as argument. Use -c ConfigFile.json");
	
	// Error message if long dashes (en dash) are used
	int i;
	for (i=0; i < *argc; i++) {
		 if ((argv[i][0] == 226) && (argv[i][1] == 128) && (argv[i][2] == 147)) {
			fprintf(stderr, "Error: Invalid arguments. En dashes are used.\n");
			return -1;
		 }
	}
	
	/* Compute the first two command line arguments */
	int c;
	std::string configPath;
	while ((c = getopt(3, argv, "c:")) != -1) {
		switch(c) {
			case 'c': 	// config found
				if(optarg){
					configPath = optarg;
				}
				else{
					throw eeros::Fault("optarg empty, no path given!");
				}
				break;
			case '?':
				if(optopt == 'c') log.trace() << "Option -" << optopt << " requires an argument.";
				else if(isprint(optopt)) log.trace() << "Unknown option `-" << optopt <<"'.";
				else log.trace() << "Unknown option character `\\x" << optopt << "'.";
				break;
			default:
				log.trace() << "ignoring: " << c;
				// ignore all other args
				break;
		}
	}
	
	
	parser = JsonParser(configPath);
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
			throw Fault("Could not add Input to HAL, signal id '" + systemInput->getId() + "' already exists!");
		}
		inputs.insert(std::pair<std::string, InputInterface*>(systemInput->getId(), systemInput));
		return true;
	}
	throw Fault("System input is null");
}
bool HAL::addOutput(OutputInterface* systemOutput) {
	if(systemOutput != nullptr) {
		if( outputs.find(systemOutput->getId()) != outputs.end() ){
			throw Fault("Could not add Output to HAL, signal id '" + systemOutput->getId() + "' already exists!");
		}
		outputs.insert(std::pair<std::string, OutputInterface*>(systemOutput->getId(), systemOutput));
		return true;
	}
	throw Fault("System output is null");
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
		throw Fault("Could not release system input '" + name + "', id not found.");
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
		throw Fault("Could not release system output '" + name + "', id not found.");
	}
}

OutputInterface* HAL::getOutput(std::string name, bool exclusive) {
	if( exclusiveReservedOutputs.find(outputs[name]) != exclusiveReservedOutputs.end() ) throw Fault("System output '" + name + "' is exclusive reserved!");
	
	if(exclusive) {
		if( nonExclusiveOutputs.find(outputs[name]) != nonExclusiveOutputs.end() ){
			throw Fault("System output '" + name + "' is already claimed as non-exclusive output!");
		}
		if(!exclusiveReservedOutputs.insert(outputs[name]).second) throw Fault("System output '" + name + "' is exclusive reserved!"); // should not fail here because already checked at the beginning
	}
	else{
		nonExclusiveOutputs.insert(outputs[name]).second;
	}
	return outputs[name];
}

Output<bool>* HAL::getLogicOutput(std::string name, bool exclusive) {
	Output<bool>* out = dynamic_cast<Output<bool>*>(outputs[name]);
	if(out == nullptr) throw Fault("Logic system output '" + name + "' not found!");
	
	if( exclusiveReservedOutputs.find(outputs[name]) != exclusiveReservedOutputs.end() ) throw Fault("Logic system output '" + name + "' is exclusive reserved!");
	
	if(exclusive) {
		if( nonExclusiveOutputs.find(outputs[name]) != nonExclusiveOutputs.end() ){
			throw Fault("Logic system output '" + name + "' is already claimed as non-exclusive output!");
		}
		if(!exclusiveReservedOutputs.insert(outputs[name]).second) throw Fault("Logic system output '" + name + "' is exclusive reserved!"); // should not fail here because already checked at the beginning
	}
	else{
		nonExclusiveOutputs.insert(outputs[name]).second;
	}
	return out;
}

ScalableOutput<double>* HAL::getScalableOutput(std::string name, bool exclusive) {
	ScalableOutput<double>* out = dynamic_cast<ScalableOutput<double>*>(outputs[name]);
	if(out == nullptr) throw Fault("Scalable system output '" + name + "' not found!");
	
	if( exclusiveReservedOutputs.find(outputs[name]) != exclusiveReservedOutputs.end() ) throw Fault("Scalable system output '" + name + "' is exclusive reserved!");
	
	if(exclusive) {
		if( nonExclusiveOutputs.find(outputs[name]) != nonExclusiveOutputs.end() ){
			throw Fault("Scalable system output '" + name + "' is already claimed as non-exclusive output!");
		}
		if(!exclusiveReservedOutputs.insert(outputs[name]).second) throw Fault("Scalable system output '" + name + "' is exclusive reserved!"); // should not fail here because already checked at the beginning
	}
	else{
		nonExclusiveOutputs.insert(outputs[name]).second;
	}
	return out;
}

InputInterface* HAL::getInput(std::string name, bool exclusive) {
	if( exclusiveReservedInputs.find(inputs[name]) != exclusiveReservedInputs.end() ) throw Fault("System input '" + name + "' is exclusive reserved!");
	
	if(exclusive) {
		if( nonExclusiveInputs.find(inputs[name]) != nonExclusiveInputs.end() ){
			throw Fault("System input '" + name + "' is already claimed as non-exclusive input!");
		}
		if(!exclusiveReservedInputs.insert(inputs[name]).second) throw Fault("System input '" + name + "' is exclusive reserved!");	// should not fail here because already checked at the beginning
	}
	else{	
		nonExclusiveInputs.insert(inputs[name]).second;
	}
	return inputs[name];
}

Input<bool>* HAL::getLogicInput(std::string name, bool exclusive) {
	Input<bool>* in = dynamic_cast<Input<bool>*>(inputs[name]);
	if(in == nullptr) throw Fault("Logic system input '" + name + "' not found!");
	
	if( exclusiveReservedInputs.find(inputs[name]) != exclusiveReservedInputs.end() ) throw Fault("Logic system input '" + name + "' is exclusive reserved!");
		
	if(exclusive) {
		if( nonExclusiveInputs.find(inputs[name]) != nonExclusiveInputs.end() ){
			throw Fault("Logic system input '" + name + "' is already claimed as non-exclusive input!");
		}
		if(!exclusiveReservedInputs.insert(inputs[name]).second) throw Fault("Logic system input '" + name + "' is exclusive reserved!");	// should not fail here because already checked at the beginning
	}
	else{	
		nonExclusiveInputs.insert(inputs[name]).second;
	}
	return in;
}

ScalableInput<double>* HAL::getScalableInput(std::string name, bool exclusive) {
	ScalableInput<double>* in = dynamic_cast<ScalableInput<double>*>(inputs[name]);
	if(in == nullptr) throw Fault("Scalable system input '" + name + "' not found!");
	
	if( exclusiveReservedInputs.find(inputs[name]) != exclusiveReservedInputs.end() ) throw Fault("Scalable system input '" + name + "' is exclusive reserved!");
	
	if(exclusive) {
		if( nonExclusiveInputs.find(inputs[name]) != nonExclusiveInputs.end() ){
			throw Fault("Scalable system input '" + name + "' is already claimed as non-exclusive input!");
		}
		if(!exclusiveReservedInputs.insert(inputs[name]).second) throw Fault("Scalable system input '" + name + "' is exclusive reserved!");	// should not fail here because already checked at the beginning
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
