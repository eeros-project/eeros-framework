#ifndef ORG_EEROS_HAL_HAL_HPP_
#define ORG_EEROS_HAL_HAL_HPP_

#include <string>
#include <map>
#include <unordered_set>
#include <eeros/hal/Input.hpp>
#include <eeros/hal/Output.hpp>
#include <eeros/hal/JsonParser.hpp>
#include <eeros/core/EEROSException.hpp>
#include <iostream>


namespace eeros {
	namespace hal {
		
		class HAL {
		public:
			OutputInterface* getOutput(std::string name, bool exclusive = true);
			Output<bool>* getLogicOutput(std::string name, bool exclusive = true);
			Output<double>* getRealOutput(std::string name, bool exclusive = true);
			InputInterface* getInput(std::string name, bool exclusive = true);
			Input<bool>* getLogicInput(std::string name, bool exclusive = true);
			Input<double>* getRealInput(std::string name, bool exclusive = true);
			
			bool addInput(InputInterface* systemInput);
			bool addOutput(OutputInterface* systemOutput);
			
			bool readConfigFromFile(std::string file);
			
			static HAL& instance();
			
			void* getOutputFeature(std::string name, std::string featureName);
			void* getOutputFeature(eeros::hal::OutputInterface * obj, std::string featureName);
			template<typename ... ArgTypes>
			void callOutputFeature(std::string name, std::string featureName, ArgTypes... args){
								
				void (*featureFunction)(eeros::hal::OutputInterface*, ArgTypes...) = reinterpret_cast<void(*)(eeros::hal::OutputInterface*, ArgTypes...)>(getOutputFeature(name, featureName));
				
				if(featureFunction == nullptr){
					throw new eeros::EEROSException("could not find method in dynamic library: " + featureName);
				}
				auto outObj = getOutput(name);
				featureFunction(outObj, args...);
			}
			
			template<typename ... ArgTypesObj>
			void callOutputFeature(eeros::hal::OutputInterface *obj, std::string featureName, ArgTypesObj... args){
								
				void (*featureFunction)(eeros::hal::OutputInterface*, ArgTypesObj...) = reinterpret_cast<void(*)(eeros::hal::OutputInterface*, ArgTypesObj...)>(getOutputFeature(obj, featureName));
				if(featureFunction == nullptr){
					throw new eeros::EEROSException("could not find method in dynamic library: " + featureName);
				}
				featureFunction(obj, args...);
				
			}
			
		private:
			HAL();
			HAL(const HAL&);
			HAL& operator=(const HAL&);
			
			bool loadModule(std::string moduleName);
			
			std::unordered_set<OutputInterface*> exclusiveReservedOutputs;
			std::unordered_set<OutputInterface*> nonExclusiveOutputs;
			std::unordered_set<InputInterface*> exclusiveReservedInputs;
			std::unordered_set<InputInterface*> nonExclusiveInputs;
			
			std::map<std::string, InputInterface*> inputs;
			std::map<std::string, OutputInterface*> outputs;
			
			std::map<std::string, void*> hwLibraries;
			JsonParser parser;
			
		};

	};
};

#endif /* ORG_EEROS_HAL_HAL_HPP_ */
