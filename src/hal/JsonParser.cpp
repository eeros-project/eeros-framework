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
		throw eeros::EEROSException(err);
	}
}

void JsonParser::createHalObjects(std::map<std::string, void*> libHandles){
	std::string library;
	std::string devHandle;
	std::string type;
	std::string chanType;
	std::string sigId;
	std::string chanUnit;
	bool inverted = false;
	double scale = 1;
	double offset = 0;
	double rangeMin = 0;
	double rangeMax = 0;
	bool channelCreated = false;
  
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
							throw eeros::EEROSException("could not load library: " + subO.string_value());
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
							// create HAL Object for channel
							//----------------------------------
							
							// check if library is loaded
							auto libIt = libHandles.find(library);
							if(!library.empty()){
								if(libIt != libHandles.end()){
									if(!devHandle.empty()){
									  
										parseChannelProperties(chanObj, &chanType, &sigId, &scale, &offset, &rangeMin, &rangeMax, &chanUnit, &inverted);
										
										if(chanType.empty()){
											chanType = type;
										}
										
										// exception for comedi Fqd
										if(chanType == "Fqd"){
											std::cout << "Fqd found" << std::endl;
											std::regex comediRegex("libcomedi[a-z.A-z]+", std::regex_constants::extended);
											if(std::regex_match(library, comediRegex)){
												std::cout << "comedi FQD required" << std::endl;
												int channelA = -1;
												int channelB = -1;
												int channelZ = -1;
												channelA = chanObj["encChannelA"].int_value();
												channelB = chanObj["encChannelB"].int_value();
												channelZ = chanObj["encChannelZ"].int_value();
												if(channelA == -1 || channelB == -1){
													throw eeros::EEROSException("no channels defined for comedi FQD signalId: '" + sigId + "'" );
												}
												createComediFqd(libIt->second, chanType, sigId, devHandle, subDevNumber, channelA, channelB, channelZ, scale, offset, rangeMin, rangeMax, chanUnit);
												channelCreated = true;
											}
										}
										
										auto typeIt = typeOfChannel.find(chanType);
										if(typeIt != typeOfChannel.end() && !channelCreated){
											if(typeIt->second == Real){
												std::cout << "create Real" << std::endl;
												createRealObject(libIt->second, chanType, sigId, devHandle, subDevNumber, channelNumber, scale, offset, rangeMin, rangeMax, chanUnit);
											}
											else if(typeIt->second == Logic){
												std::cout << "create Logic" << std::endl;
												createLogicObject(libIt->second, chanType, sigId, devHandle, subDevNumber, channelNumber, inverted);
											}
										}
										else{
											if(!channelCreated){
												throw eeros::EEROSException("undefined type: " + chanType + " for " + sigId);
											}
										}
										sigId.clear();
										chanType.clear();
										chanUnit.clear();
										inverted = false;
										scale = 1;
										offset = 0;
										rangeMin = 0;
										rangeMax = 0;
										channelCreated = false;
									}
									else{
										throw eeros::EEROSException("no device handle defined for " + subDevParam);
									}
								}
							}
							else{
								throw eeros::EEROSException("no library defined for " + subDevParam);
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
		throw eeros::EEROSException("No parsed HAL root object");
	}
}

void JsonParser::parseChannelProperties(ucl::Ucl chanObj, std::string* chanType, std::string* sigId, double* scale, double* offset, double* rangeMin, double* rangeMax, std::string *chanUnit, bool *inverted){
	*sigId = chanObj["signalId"].string_value();
	std::cout << "\t\t\tsignalId: " << *sigId << std::endl;
	
	std::cout << "\t\t\ttype: " << chanObj["type"].string_value() << std::endl;
	*chanType = chanObj["type"].string_value();

	*inverted = chanObj["inverted"].bool_value();
	
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
	bool noRangeLimit = false;
	double tmpScale = 1.0;
	double tmpOffset = 0.0;
	double tmpRangeMinIn = 0.0;
	double tmpRangeMaxIn = 0.0;
	double tmpRangeMaxOut = 0.0;
	double tmpRangeMinOut = 0.0;
	bool firstScale = true;
	
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
				tmpOffset = minMax.number_value();
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
				if(obj["range"].at(pos)["noLimit"].bool_value() == true){
					noRangeLimit = true;
					*rangeMin = std::numeric_limits<double>::min();
					*rangeMax = std::numeric_limits<double>::max();
				}
			}
			i++;
		}
		if(pos == -1){
			throw eeros::EEROSException("no range values found for " + id);
		}
		
		if(noRangeLimit && !setScaleDirect){
			tmpRangeMaxOut = maxOut;
			tmpRangeMaxIn = maxIn;
			tmpRangeMinIn = minIn;
			tmpRangeMinOut = minOut;
		}
		
		if(!noRangeLimit){
			tmpRangeMinIn = obj["range"].at(pos)["minIn"].number_value();
			tmpRangeMaxIn = obj["range"].at(pos)["maxIn"].number_value();
			tmpRangeMaxOut = obj["range"].at(pos)["maxOut"].number_value();
			tmpRangeMinOut = obj["range"].at(pos)["minOut"].number_value();
		}
		
			if(!setScaleDirect){
				//calculate scale, offset
				lsb = (maxOut - minOut)/(maxIn - minIn);
				xMin = minIn - (minOut + tmpRangeMaxOut) / lsb;
				xMax = maxIn - (maxOut + tmpRangeMinOut) / lsb;
				m = (xMax - xMin) / ( fabs(tmpRangeMinOut) + fabs(tmpRangeMaxOut) );
				tmpScale = 1 / m;
				tmpOffset = -(tmpRangeMaxOut + xMin * tmpScale);
			}
			
			if(firstScale){
				firstScale = false;
				*rangeMin = tmpRangeMinIn;
				*rangeMax = tmpRangeMaxIn;
			}
			
			// calculate new range for next input scale before applying new scale
			// check if new range is reduced by new block (min value), replace if necessary
			std::cout << "rangeMinIn: " << tmpRangeMinIn << "\trangeMaxIn: " << tmpRangeMaxIn << std::endl;
			std::cout << "rangeMinOut: " << tmpRangeMinOut << "\trangeMaxOut: " << tmpRangeMaxOut << std::endl;
			std::cout << "oldMin: " << *rangeMin << "\toldMax: " << *rangeMax << std::endl;
			std::cout << "checkMin: " << ((tmpRangeMinIn - *offset) / (*scale)) << std::endl;
			
		if(!noRangeLimit){	
			if( ((tmpRangeMinIn - *offset) / (*scale)) > (*rangeMin) ){
				*rangeMin = (tmpRangeMinIn - *offset) / (*scale);
				std::cout << "new rangeMin: " << *rangeMin << std::endl;
			}
			
			std::cout << "checkMax: " << ((tmpRangeMaxIn - *offset) / (*scale)) << std::endl;
			// check range max value, replace if necessary
			if( ((tmpRangeMaxIn - *offset) / (*scale)) < (*rangeMax) ){
				*rangeMax = (tmpRangeMaxIn - *offset) / (*scale);
				std::cout << "new rangeMax: " << *rangeMax << std::endl;
			}
		}
			
		// apply new scale
		std::cout << "tmpScale: " << tmpScale << "\ttmpOffset: " << tmpOffset << std::endl;
		std::cout << "oldScale: " << *scale << "\toldOffset: " << *offset << std::endl;
		*offset = (*offset) * tmpScale + tmpOffset;
		*scale = (*scale) * tmpScale;
		std::cout << "scale: " << *scale << "\toffset: " << (*offset) << std::endl;
		
		if(!noRangeLimit){
			std::cout << "checkMinOut: " << ((tmpRangeMinOut - *offset) / (*scale)) << std::endl;
			std::cout << "checkMaxOut: " << ((tmpRangeMaxOut - *offset) / (*scale)) << std::endl;
			// calculate new output range with new scale
			// check out max/min
			if( ((tmpRangeMinOut - *offset) / (*scale)) > (*rangeMin) ){
				*rangeMin = ((tmpRangeMinOut - *offset) / (*scale));
				std::cout << "new rangeMin: " << *rangeMin << std::endl;
			}
			if( ((tmpRangeMaxOut - *offset) / (*scale)) < (*rangeMax) ){
				*rangeMax = ((tmpRangeMaxOut - *offset) / (*scale));
				std::cout << "new rangeMax: " << *rangeMax << std::endl;
			}
			
			if( !std::isfinite(*rangeMin) || !std::isfinite(*rangeMin) ){
				throw eeros::EEROSException("config for range is invalid, id: " + id);
			}
		}
		if( !std::isfinite(*scale) || !std::isfinite(*offset) ){
			throw eeros::EEROSException("config for scale is invalid, id: " + id);
		}
		
		setScaleDirect = false;
		noRangeLimit = false;
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
		tmpRangeMinIn = 0.0;
		tmpRangeMaxIn = 0.0;
		tmpRangeMaxOut = 0.0;
		tmpRangeMinOut = 0.0;
	}
}

void JsonParser::createLogicObject(void *libHandle, std::string type, std::string id, std::string devHandle, uint32_t subDevNumber, uint32_t channelNumber, bool inverted){
	HAL& hal = HAL::instance();
	
	if(libHandle == nullptr || type.empty() || id.empty() || devHandle.empty()){
		throw std::invalid_argument("could not create object");
	}
	
	std::string createStr = "create" + type;
	void *createHandle = dlsym(libHandle, createStr.c_str());
	if(createHandle == nullptr){
		std::cout << "could not find createMethod: " << dlerror() << std::endl;
		throw eeros::EEROSException("could not find method in dynamic library");
	}
	auto dirIt = directionOfChannel.find(type);
	if(dirIt != directionOfChannel.end()){
		if(dirIt->second == In){
			std::cout << "createIn" << std::endl;
			Input<bool> *halObj = reinterpret_cast<Input<bool> *(*)(std::string, void*, std::string, uint32_t, uint32_t, bool)>(createHandle)(id, libHandle, devHandle, subDevNumber, channelNumber, inverted);
			hal.addInput(halObj);
		}
		else if(dirIt->second == Out){
			std::cout << "createOut" << std::endl;
			Output<bool> *halObj = reinterpret_cast<Output<bool> *(*)(std::string, void*, std::string, uint32_t, uint32_t, bool)>(createHandle)(id, libHandle, devHandle, subDevNumber, channelNumber, inverted);
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

void JsonParser::createRealObject(void *libHandle, std::string type, std::string id, std::string devHandle, uint32_t subDevNumber, uint32_t channelNumber, double scale, double offset, double rangeMin, double rangeMax, std::string unit){
	HAL& hal = HAL::instance();
	
	if(libHandle == nullptr || type.empty() || id.empty() || devHandle.empty()){
		throw std::invalid_argument("could not create object");
	}
	
	std::string createStr = "create" + type;
	void *createHandle = dlsym(libHandle, createStr.c_str());
	if(createHandle == nullptr){
		std::cout << "could not find createMethod: " << dlerror() << std::endl;
		throw eeros::EEROSException("could not find method in dynamic library");
	}
	auto dirIt = directionOfChannel.find(type);
	if(dirIt != directionOfChannel.end()){
		if(dirIt->second == In){
			std::cout << "createIn" << std::endl;
			ScalableInput<double> *halObj = reinterpret_cast<ScalableInput<double> *(*)(std::string, void*, std::string, uint32_t, uint32_t, double, double, double, double, std::string)>(createHandle)(id, libHandle, devHandle, subDevNumber, channelNumber, scale, offset, rangeMin, rangeMax, unit);
			hal.addInput(halObj);
		}
		else if(dirIt->second == Out){
			std::cout << "createOut" << std::endl;
			ScalableOutput<double> *halObj = reinterpret_cast<ScalableOutput<double> *(*)(std::string, void*, std::string, uint32_t, uint32_t, double, double, double, double, std::string)>(createHandle)(id, libHandle, devHandle, subDevNumber, channelNumber, scale, offset, rangeMin, rangeMax, unit);
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

void JsonParser::createComediFqd(void *libHandle, std::string type, std::string id, std::string devHandle, uint32_t subDevNumber, uint32_t channelA, uint32_t channelB, uint32_t channelZ, double scale, double offset, double rangeMin, double rangeMax, std::string unit){
	HAL& hal = HAL::instance();
	
	if(libHandle == nullptr || type.empty() || id.empty() || devHandle.empty()){
		throw std::invalid_argument("could not create object");
	}
	
	std::string createStr = "create" + type;
	void *createHandle = dlsym(libHandle, createStr.c_str());
	if(createHandle == nullptr){
		std::cout << "could not find createMethod: " << dlerror() << std::endl;
		throw eeros::EEROSException("could not find method in dynamic library");
	}
	auto dirIt = directionOfChannel.find(type);
	if(dirIt != directionOfChannel.end()){
		if(dirIt->second == In){
			ScalableInput<double> *halObj = reinterpret_cast<ScalableInput<double> *(*)(std::string, void*, std::string, uint32_t, uint32_t, uint32_t, uint32_t, double, double, double, double, std::string)>(createHandle)(id, libHandle, devHandle, subDevNumber, channelA, channelB, channelZ, scale, offset, rangeMin, rangeMax, unit);
			hal.addInput(halObj);
		}
		else{
			throw eeros::EEROSException("wrong direction for comedi FQD channel " + id);
		}
	}
	else{
		throw eeros::EEROSException("undefined direction for channel " + id);
	}
}