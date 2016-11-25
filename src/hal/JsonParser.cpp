#include <eeros/hal/JsonParser.hpp>
#include <eeros/core/EEROSException.hpp>
#include <regex>
#include <dlfcn.h>
#include <eeros/hal/PeripheralOutput.hpp>
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
					for(const auto &chanO : subDevO){
						std::regex chanRegex("channel[0-9]+", std::regex_constants::extended);
						std::string subDevParam = chanO.key();
						
						if(std::regex_match(subDevParam, chanRegex)){
							std::cout << "\t\t" << subDevParam << std::endl;
							
							auto chanObj = chanO;
							
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
												type = chanProp.string_value();
											}
											if(chanProp.key() == "signalId"){
												std::cout << "\t\t\tsignalId: " << chanProp.string_value() << std::endl;
												//TODO create function which can create string createDigOut from type
												//TODO: change when using HAL Object: only for test purposes
												int subDevNumber = std::stoi(subDevO.key().substr(9, subDevO.key().length()));
												int channelNumber = std::stoi(chanObj.key().substr(7, chanObj.key().length()));
												
												createObject(libIt->second, type, chanProp.string_value(), devHandle, subDevNumber, channelNumber);
											}
										}
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

void JsonParser::createObject(void *libHandle, std::string type, std::string id, std::string devHandle, uint32_t subDevNumber, uint32_t channelNumber){
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
	auto typeIt = typeOfChannel.find(type);
	if(dirIt != directionOfChannel.end()){
		if(typeIt != typeOfChannel.end()){
			//TODO add generic for type
			
			if(dirIt->second == Input){
				std::cout << "createIn" << std::endl;
				PeripheralInput<bool> *halObj = reinterpret_cast<PeripheralInput<bool> *(*)(std::string, std::string, uint32_t, uint32_t)>(createHandle)(id, devHandle, subDevNumber, channelNumber);
				hal.addPeripheralInput(halObj);
			}
			else if(dirIt->second == Output){
				std::cout << "createOut" << std::endl;
				PeripheralOutput<bool> *halObj = reinterpret_cast<PeripheralOutput<bool> *(*)(std::string, std::string, uint32_t, uint32_t)>(createHandle)(id, devHandle, subDevNumber, channelNumber);
				hal.addPeripheralOutput(halObj);
			}
			else{
				throw eeros::EEROSException("undefined direction for channel " + id);
			}
		}
		else{
			throw eeros::EEROSException("undefined type for channel " + id);
		}
	}
	else{
		throw eeros::EEROSException("undefined direction for channel " + id);
	}
}

