#include <eeros/hal/JsonParser.hpp>
#include <eeros/core/EEROSException.hpp>
#include <regex>
#include <dlfcn.h>

using namespace eeros;
using namespace eeros::hal;

JsonParser::JsonParser(const char *filePath){
	std::string err;
	
	halRootObj = ucl::Ucl::parse_from_file(filePath, err);
	if(!err.empty()){
		throw new eeros::EEROSException(err);
	}
}

void JsonParser::createHalObjects(){
	if (halRootObj) {
		std::cout << "halRootObj found" << std::endl;
		//std::cout << obj.dump(UCL_EMIT_CONFIG) << std::endl;
		  
		for (const auto &o : halRootObj) {
			auto devObj = o;
			for(const auto &subO : devObj){
				void *libFlinkHandle = nullptr;
				void *createDevHandle = nullptr;
				std::regex sdRegex("subdevice[0-9]+", std::regex_constants::extended);
				if(subO.key() == "library"){
					std::cout << "\tload library for " << subO.string_value() << std::endl;
					
					if(subO.string_value() == "libflinkeeros.so"){
						std::cout << "will load flinkeeros" << std::endl;
						libFlinkHandle = dlopen("libflinkeeros.so", RTLD_NOW);//dlopen(subO.string_value().c_str(), RTLD_NOW);
						if(libFlinkHandle == nullptr){
							std::cout << "could not load library: " << dlerror() << std::endl;
							throw new eeros::EEROSException("could not load library: " + subO.string_value());
						}
						else{
							std::cout << "lib successfully loaded" << std::endl;
						}
					}
					//---------------------------------------------
					//TODO load driver/hw library for driver field
					//---------------------------------------------
				}
				else if(subO.key() == "devHandle"){
					std::cout << "\tdev Handle is " << subO.string_value() << std::endl;
					//TODO if flink, now just create the device
					
				}
				else if(std::regex_match(subO.key(), sdRegex)){
					std::cout << "\t" << subO.key() << std::endl;
					auto subDevO = subO;
					for(const auto &chanO : subDevO){
						std::regex chanRegex("channel[0-9]+", std::regex_constants::extended);
						std::string subDevParam = chanO.key();
						
						if(subDevParam == "type"){
							std::cout << "\t\ttype: " << chanO.string_value() << std::endl;
						}
						else if(std::regex_match(subDevParam, chanRegex)){
							std::cout << "\t\t" << subDevParam << std::endl;
							
							auto chanObj = chanO;
							
							//----------------------------------
							//TODO create HAL Object for channel
							//----------------------------------
							for(const auto &chanProp : chanObj){
								if(chanProp.key() == "signalId"){
									std::cout << "\t\t\tsignalId: " << chanProp.string_value() << std::endl;
								}
							}
							
							//TODO: change when using HAL Object: only for test purposes
							std::string subDevStr = subDevO.key();
							int subDevNumber = std::stoi(subDevStr.substr(9, subDevStr.length()));
							std::cout << "subDevNum: " << subDevNumber << std::endl;
							
							std::string chanStr = chanObj.key();
							int channelNumber = std::stoi(chanStr.substr(7, chanStr.length()));
							std::cout << "channelNum: " << channelNumber << std::endl;
							//TODO: upper lines only temporary
						}
						else{
							std::cout << "\t\tno valid key: " << subDevParam << std::endl;
						}
						
					}
				}
			}
			if(o.key() == "device0"){
				std::cout << "device0 found" << std::endl;
			}
			else{
				std::cout << "device0 not found" << std::endl;
			}
			
			
			//std::cout << o.dump(UCL_EMIT_CONFIG) << std::endl;
		}
	}
	else{
		throw new eeros::EEROSException("No parsed HAL root object");
	}
}