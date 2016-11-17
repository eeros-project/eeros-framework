#include <eeros/hal/JsonParser.hpp>
#include <eeros/core/EEROSException.hpp>
#include <regex>
#include <dlfcn.h>
#include <eeros/hal/PeripheralOutput.hpp>

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
										
					if(subO.string_value() == "libflinkeeros.so"){
						std::cout << "will load flinkeeros" << std::endl;
						
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
						
						if(subDevParam == "type"){
							std::cout << "\t\ttype: " << chanO.string_value() << std::endl;
							type = chanO.string_value();
						}
						else if(std::regex_match(subDevParam, chanRegex)){
							std::cout << "\t\t" << subDevParam << std::endl;
							
							auto chanObj = chanO;
							
							//----------------------------------
							//TODO create HAL Object for channel
							//----------------------------------
							// check if device already exists and library is loaded?
							
							auto libIt = libHandles.find(library);
							if(!library.empty() && (libIt != libHandles.end()) ){
								if(!devHandle.empty()){
									for(const auto &chanProp : chanObj){
										if(chanProp.key() == "signalId"){
											std::cout << "\t\t\tsignalId: " << chanProp.string_value() << std::endl;
											//TODO create function which can create string createDigOut from type
											//TODO: change when using HAL Object: only for test purposes
											std::string subDevStr = subDevO.key();
											int subDevNumber = std::stoi(subDevStr.substr(9, subDevStr.length()));
											std::cout << "subDevNum: " << subDevNumber << std::endl;
											
											std::string chanStr = chanObj.key();
											int channelNumber = std::stoi(chanStr.substr(7, chanStr.length()));
											std::cout << "channelNum: " << channelNumber << std::endl;
											//TODO: upper lines only temporary
											std::string createStr = "create" + type;
											std::cout << "method string: " << createStr << std::endl;
											typedef PeripheralOutput<bool> * createSignature_t(std::string, std::string, uint32_t, uint32_t, bool);
											void *createHandle = dlsym(libIt->second, createStr.c_str());
											if(createHandle == nullptr){
												std::cout << "could not find createMethod: " << dlerror() << std::endl;
												throw new eeros::EEROSException("could not find method in dynamic library");
											}
											PeripheralOutput<bool> *halObj = reinterpret_cast<PeripheralOutput<bool> *(*)(std::string, std::string, uint32_t, uint32_t)>(createHandle)(chanProp.string_value(), devHandle, subDevNumber, channelNumber);
// 											PeripheralOutput<bool> *halObj = reinterpret_cast<PeripheralOutput<bool> *(*)()>(createHandle)(chanProp.string_value(), devHandle, subDevNumber, channelNumber);
// 											createSignature_t* halFun = reinterpret_cast<createSignature_t*>(createHandle);
											
// 											PeripheralOutput<bool> *halObj = halFun(chanProp.string_value(), devHandle, subDevNumber, channelNumber, false);



											/*
											typedef PeripheralOutput<bool> * createSignature_t(std::string, std::string, uint32_t, uint32_t, bool);
											void *createHandle = dlsym(libIt->second, createStr.c_str());
											if(createHandle == nullptr){
												std::cout << "could not find createMethod: " << dlerror() << std::endl;
												throw new eeros::EEROSException("could not find method in dynamic library");
											}
// 											PeripheralOutput<bool> *halObj = reinterpret_cast<PeripheralOutput<bool> *(*)()>(createHandle)(chanProp.string_value(), devHandle, subDevNumber, channelNumber);
											createSignature_t* halFun = reinterpret_cast<createSignature_t*>(createHandle);
											
											PeripheralOutput<bool> *halObj = halFun(chanProp.string_value(), devHandle, subDevNumber, channelNumber, false);*/
											
// 											halObj->set(true);
										}
									}
								}
								else{
									std::cout << "no device handle defined for " << subDevParam << std::endl;
									throw new eeros::EEROSException("no device handle defined for " + subDevParam);
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