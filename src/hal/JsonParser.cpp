#include <eeros/hal/JsonParser.hpp>
#include <eeros/core/EEROSException.hpp>
#include <regex>
#include <dlfcn.h>
#include <eeros/hal/Output.hpp>
#include <eeros/hal/Input.hpp>
#include <eeros/hal/ScalableInput.hpp>
#include <eeros/hal/ScalableOutput.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/HALFeatures.hpp>
#include <stdexcept>

using namespace eeros;
using namespace eeros::hal;

JsonParser::JsonParser(){ }

JsonParser::JsonParser(std::string filePath){
	std::string err;
	
	halRootObj = ucl::Ucl::parse_from_file(filePath.c_str(), err);
	if(!err.empty()){
		throw new eeros::EEROSException(err);
	}
}

void JsonParser::createHalObjects(std::map<std::string, void*> libHandles){
	std::string library;
	std::string devHandle;
	std::string type;
	std::string chanType;
	std::string sigId;
	std::string chanUnit;
	double scale = 1;
	double offset = 0;
  
	if (halRootObj) {
		std::cout << "halRootObj found" << std::endl;
		//std::cout << obj.dump(UCL_EMIT_CONFIG) << std::endl;
		  
		for (const auto &o : halRootObj) {
			auto devObj = o;
			for(const auto &subO : devObj){
				void *createDevHandle = nullptr;
				std::regex sdRegex("subdevice[0-9]+", std::regex_constants::extended);
				if(subO.key() == "library"){
					std::cout << "\tload library for " << subO.string_value() << std::endl;
					library = subO.string_value();
					auto libIt = libHandles.find(library);
					// if library not already in map of opened libraries -> try to open it
					if(libIt == libHandles.end()){
						libHandles[library] = dlopen(library.c_str(), RTLD_NOW); // try to load the given library and add it to libHandles map
						auto libIt = libHandles.find(library);	
						if(libIt->second == nullptr){
							libHandles.erase(libIt);
							std::cout << "could not load library: " << dlerror() << std::endl;
							throw new eeros::EEROSException("could not load library: " + subO.string_value());
						}
						else{
							std::cout << "lib successfully loaded" << std::endl;
						}
					}
				}
				else if(subO.key() == "devHandle"){
					devHandle = subO.string_value();
					std::cout << "\tdev Handle is " << subO.string_value() << std::endl;
				}
				else if(std::regex_match(subO.key(), sdRegex)){
					std::cout << "\t" << subO.key() << std::endl;
					auto subDevO = subO;
					int subDevNumber = std::stoi(subDevO.key().substr(9, subDevO.key().length()));
					for(const auto &chanO : subDevO){
						std::regex chanRegex("channel[0-9]+", std::regex_constants::extended);
						std::string subDevParam = chanO.key();
						std::cout << "." << std::endl;
						if(chanO.key() == "type"){
							std::cout << "\t\t\ttype: " << chanO.string_value() << std::endl;
							type = chanO.string_value();
						}
						else if(std::regex_match(subDevParam, chanRegex)){
							std::cout << "\t\t" << subDevParam << std::endl;
							
							auto chanObj = chanO;
							int channelNumber = std::stoi(chanObj.key().substr(7, chanObj.key().length()));
						
							//----------------------------------
							//TODO create HAL Object for channel
							//----------------------------------
							// check if device already exists and library is loaded?
							
							auto libIt = libHandles.find(library);
							if(!library.empty()){
								if(libIt != libHandles.end()){
									if(!devHandle.empty()){
										for(const auto &chanProp : chanObj){
											if(chanProp.key() == "type"){
												std::cout << "\t\t\ttype: " << chanProp.string_value() << std::endl;
												chanType = chanProp.string_value();
											}
											if(chanProp.key() == "scale"){
												for(const auto &scaleProp: chanProp){
													std::cout << "sc: " << scaleProp.key() << std::endl;
												}
												//TODO
												std::cout << "scale: " << chanProp.at(0).string_value() << std::endl;
// 												std::cout << "length: " << chanProp.len() << std::endl;
											}
											if(chanProp.key() == "range"){
												//TODO
											}
											if(chanProp.key() == "signalId"){
												std::cout << "\t\t\tsignalId: " << chanProp.string_value() << std::endl;
												sigId = chanProp.string_value();
												
											}
											if(chanProp.key() == "unit"){
												chanUnit = chanProp.string_value();
											}
										}
										if(chanType.empty()){
											chanType = type;
										}
										auto typeIt = typeOfChannel.find(chanType);
										if(typeIt != typeOfChannel.end()){
											if(typeIt->second == Real){
												std::cout << "create Real" << std::endl;
												createRealObject(libIt->second, chanType, sigId, devHandle, subDevNumber, channelNumber, scale, offset, chanUnit);
											}
											else if(typeIt->second == Logic){
												std::cout << "create Logic" << std::endl;
												createLogicObject(libIt->second, chanType, sigId, devHandle, subDevNumber, channelNumber);
											}
										}
										else{
											throw eeros::EEROSException("undefined type: " + chanType + "for " + sigId);
										}
										sigId.clear();
										chanType.clear();
										chanUnit.clear();
										scale = 1;
										offset = 0;
									}
									else{
										std::cout << "no device handle defined for " << subDevParam << std::endl;
										throw new eeros::EEROSException("no device handle defined for " + subDevParam);
									}
								}
							}
							else{
								std::cout << "no library defined for " << subDevParam << std::endl;
								throw new eeros::EEROSException("no library defined for " + subDevParam);
							}
							// check if end of lib
							
							
							
							
							
						}
						else{
							std::cout << "\t\tno valid key: " << subDevParam << std::endl;
						}
					}
					type.clear();
				}
			}
			if(o.key() == "device0"){
				std::cout << "device0 found" << std::endl;
			}
			else{
				std::cout << "device0 not found" << std::endl;
			}
			
			library.clear();			//make sure that no settings are handed over to next object
			devHandle.clear();
		}
	}
	else{
		throw new eeros::EEROSException("No parsed HAL root object");
	}
}

void JsonParser::createLogicObject(void *libHandle, std::string type, std::string id, std::string devHandle, uint32_t subDevNumber, uint32_t channelNumber){
	HAL& hal = HAL::instance();
	
	if(libHandle == nullptr || type.empty() || id.empty() || devHandle.empty()){
		throw std::invalid_argument("could not create object");
	}
	
	std::string createStr = "create" + type;
	void *createHandle = dlsym(libHandle, createStr.c_str());
	if(createHandle == nullptr){
		std::cout << "could not find createMethod: " << dlerror() << std::endl;
		throw new eeros::EEROSException("could not find method in dynamic library");
	}
	auto dirIt = directionOfChannel.find(type);
	if(dirIt != directionOfChannel.end()){
	//TODO add generic for type
		if(dirIt->second == In){
			std::cout << "createIn" << std::endl;
			Input<bool> *halObj = reinterpret_cast<Input<bool> *(*)(std::string, std::string, uint32_t, uint32_t)>(createHandle)(id, devHandle, subDevNumber, channelNumber);
			hal.addInput(halObj);
		}
		else if(dirIt->second == Out){
			std::cout << "createOut" << std::endl;
			Output<bool> *halObj = reinterpret_cast<Output<bool> *(*)(std::string, std::string, uint32_t, uint32_t)>(createHandle)(id, devHandle, subDevNumber, channelNumber);
			hal.addOutput(halObj);
		}
		else{
			throw eeros::EEROSException("undefined direction for channel " + id);
		}
	}
	else{
		throw eeros::EEROSException("undefined direction for channel " + id);
	}
}

void JsonParser::createRealObject(void *libHandle, std::string type, std::string id, std::string devHandle, uint32_t subDevNumber, uint32_t channelNumber, double scale, double offset, std::string unit){
	HAL& hal = HAL::instance();
	
	if(libHandle == nullptr || type.empty() || id.empty() || devHandle.empty()){
		throw std::invalid_argument("could not create object");
	}
	
	std::string createStr = "create" + type;
	void *createHandle = dlsym(libHandle, createStr.c_str());
	if(createHandle == nullptr){
		std::cout << "could not find createMethod: " << dlerror() << std::endl;
		throw new eeros::EEROSException("could not find method in dynamic library");
	}
	auto dirIt = directionOfChannel.find(type);
	if(dirIt != directionOfChannel.end()){
	//TODO add generic for type
		if(dirIt->second == In){
			std::cout << "createIn" << std::endl;
			ScalableInput<double> *halObj = reinterpret_cast<ScalableInput<double> *(*)(std::string, std::string, uint32_t, uint32_t, double, double, std::string)>(createHandle)(id, devHandle, subDevNumber, channelNumber, scale, offset, unit);
			hal.addInput(halObj);
		}
		else if(dirIt->second == Out){
			std::cout << "createOut" << std::endl;
			ScalableOutput<double> *halObj = reinterpret_cast<ScalableOutput<double> *(*)(std::string, std::string, uint32_t, uint32_t, double, double, std::string)>(createHandle)(id, devHandle, subDevNumber, channelNumber, scale, offset, unit);
			hal.addOutput(halObj);
		}
		else{
			throw eeros::EEROSException("undefined direction for channel " + id);
		}
	}
	else{
		throw eeros::EEROSException("undefined direction for channel " + id);
	}
}
