#include <eeros/hal/JsonParser.hpp>
#include <eeros/core/EEROSException.hpp>
#include <regex>
#include <cmath>
#include <dlfcn.h>
#include <eeros/hal/Output.hpp>
#include <eeros/hal/Input.hpp>
#include <eeros/hal/ScalableInput.hpp>
#include <eeros/hal/ScalableOutput.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/HALFeatures.hpp>
#include <stdexcept>
#include <ucl.h>

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
	double rangeMin = 0;
	double rangeMax = 0;
  
	if (halRootObj) {
		  
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
									  
										parseChannelProperties(chanObj, &chanType, &sigId, &scale, &offset, &rangeMin, &rangeMax, &chanUnit);
										
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
										rangeMin = 0;
										rangeMax = 0;
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

void JsonParser::parseChannelProperties(ucl::Ucl chanObj, std::string* chanType, std::string* sigId, double* scale, double* offset, double* rangeMin, double* rangeMax, std::string *chanUnit){
	double min = -10.0;
	double max = 10.0;
	
	*sigId = chanObj["signalId"].string_value();
	std::cout << "\t\t\tsignalId: " << *sigId << std::endl;
	
	std::cout << "\t\t\ttype: " << chanObj["type"].string_value() << std::endl;
	*chanType = chanObj["type"].string_value();

	for(const auto &chanProp : chanObj){
		if(chanProp.key() == "unit"){
			*chanUnit = chanProp.string_value();
		}
		
	}
	
	calcScale(chanObj, scale, offset, rangeMin, rangeMax);
}


void JsonParser::calcScale(ucl::Ucl obj, double *scale, double *offset, double *rangeMin, double *rangeMax){
	if(scale == nullptr || offset == nullptr || rangeMin == nullptr || rangeMax == nullptr){
		throw std::invalid_argument("parameter nullptr");
	}
	*scale = 1.0;
	*offset = 0.0;
	*rangeMax = 0.0;
	*rangeMin = 0.0;
  
	double minIn = 0.0;
	double maxIn = 0.0;
	double minOut = 0.0;
	double maxOut = 0.0;
	double lsb = 0.0;
	double xMin = 0.0;
	double xMax = 0.0;
	double m = 0.0;
	std::string id;
	bool setScaleDirect = false;
	double tmpScale = 1.0;
	double tmpOffset = 0.0;
	
	for(const auto &scaleProp: obj["scale"]){
		
		for(const auto &minMax: scaleProp){
			if(minMax.key() == "id"){
				id = minMax.string_value();
			}
			else if(minMax.key() == "minIn"){
				minIn = minMax.number_value();
			}
			else if(minMax.key() == "maxIn"){
				maxIn = minMax.number_value();
			}
			else if(minMax.key() == "minOut"){
				minOut = minMax.number_value();
			}
			else if(minMax.key() == "maxOut"){
				maxOut = minMax.number_value();
			}
			else if(minMax.key() == "scale"){
				tmpScale = minMax.number_value();
				setScaleDirect = true;
			}
			else if(minMax.key() == "offset"){
				tmpOffset = -minMax.number_value();
			}
			else{
				throw eeros::EEROSException("unknown scale property in id: " + id);
			}
		}
		
		int i = 0;
		int pos = -1;
		for(const auto &rangeIt: obj["range"]){
			if(rangeIt["id"].string_value() == id){
				pos = i;
			}
			i++;
		}
		if(pos == -1){
			throw eeros::EEROSException("no range values found for " + id);
		}
		
		double tmpRangeMinIn = obj["range"].at(pos)["minIn"].number_value();
		double tmpRangeMaxIn = obj["range"].at(pos)["maxIn"].number_value();
		double tmpRangeOutMax = obj["range"].at(pos)["maxOut"].number_value();
		double tmpRangeOutMin = obj["range"].at(pos)["minOut"].number_value();
			
		
		if(!setScaleDirect){
			//calculate scale, offset
			lsb = (maxOut - minOut)/(maxIn - minIn);
			xMin = minIn - (minOut + tmpRangeOutMax) / lsb;
			xMax = maxIn - (maxOut + tmpRangeOutMin) / lsb;
			m = (xMax - xMin) / ( fabs(tmpRangeOutMin) + fabs(tmpRangeOutMax) );
			tmpScale = 1 / m;
			tmpOffset = -(tmpRangeOutMax + xMin * tmpScale);
		}
		
		// calculate new range before applying new scale
		// check if new range is reduced by new block (min value), replace if necessary
		if( ((tmpRangeMinIn - *offset)/(*scale)) < (*rangeMin) ){
			//TODO check for range Min Out
			*rangeMin = (tmpRangeMinIn - *offset)/(*scale);
		}
		// check range max value, replace if necessary
		if( ((tmpRangeMaxIn - *offset)/(*scale)) > (*rangeMax) ){
			*rangeMax = (tmpRangeMaxIn - *offset)/(*scale);
		}
			
		// apply new scale
		std::cout << "tmpScale: " << tmpScale << "\ttmpOffset: " << tmpOffset << std::endl;
		std::cout << "oldScale: " << *scale << "\toldOffset: " << *offset << std::endl;
		*offset = (*offset) * tmpScale + tmpOffset;
// 		*offset = (*offset) * tmpScale + tmpOffset * (*scale);
		*scale = (*scale) * tmpScale;
		std::cout << "scale: " << *scale << "\toffset: " << (*offset) << std::endl;
		
		if( !std::isfinite(*scale) || !std::isfinite(*offset) || !std::isfinite(*rangeMin) || !std::isfinite(*rangeMin) ){
			throw eeros::EEROSException("config for scale or range is invalid, id: " + id);
		}
		
		setScaleDirect = false;
		id.clear();
		minIn = 0.0;
		maxIn = 0.0;
		minOut = 0.0;
		maxOut = 0.0;
		lsb = 0.0;
		xMin = 0.0;
		xMax = 0.0;
		m = 0.0;
		tmpScale = 1.0;
		tmpOffset = 0.0;
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
