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
			void releaseInput(std::string name);
			void releaseOutput(std::string name);
			
			bool addInput(InputInterface* systemInput);
			bool addOutput(OutputInterface* systemOutput);
			
			bool readConfigFromFile(std::string file);
			
			static HAL& instance();
						
			template<typename ... ArgTypes>
			void callOutputFeature(std::string name, std::string featureName, ArgTypes... args){
				
				void (*featureFunction)(OutputInterface*, ArgTypes...) = reinterpret_cast<void(*)(OutputInterface*, ArgTypes...)>(getOutputFeature(name, featureName));
				
				if(featureFunction == nullptr){
					throw EEROSException("could not find method in dynamic library: " + featureName);
				}
				auto outObj = outputs[name];			//TODO should we allow that!? or can a user do something bad with a feature Function!?
				featureFunction(outObj, args...);
			}
			
			template<typename ... ArgTypesObj>
			void callOutputFeature(OutputInterface *obj, std::string featureName, ArgTypesObj... args){
				
				void (*featureFunction)(OutputInterface*, ArgTypesObj...) = reinterpret_cast<void(*)(OutputInterface*, ArgTypesObj...)>(getOutputFeature(obj, featureName));
				if(featureFunction == nullptr){
					throw EEROSException("could not find method in dynamic library: " + featureName);
				}
				featureFunction(obj, args...);
			}
			
			template<typename ... ArgTypesStrIn>
			void callInputFeature(std::string name, std::string featureName, ArgTypesStrIn... args){
				
				void (*featureFunction)(InputInterface*, ArgTypesStrIn...) = reinterpret_cast<void(*)(InputInterface*, ArgTypesStrIn...)>(getInputFeature(name, featureName));
				
				if(featureFunction == nullptr){
					throw EEROSException("could not find method in dynamic library: " + featureName);
				}
				auto inObj = inputs[name];			//TODO should we allow that!? or can a user do something bad with a feature Function!?
				featureFunction(inObj, args...);
			}
			
			template<typename ... ArgTypesIn>
			void callInputFeature(InputInterface *obj, std::string featureName, ArgTypesIn... args){
				
				void (*featureFunction)(InputInterface*, ArgTypesIn...) = reinterpret_cast<void(*)(InputInterface*, ArgTypesIn...)>(getInputFeature(obj, featureName));
				if(featureFunction == nullptr){
					throw EEROSException("could not find method in dynamic library: " + featureName);
				}
				featureFunction(obj, args...);
			}
			
		private:
			HAL();
			HAL(const HAL&);
			HAL& operator=(const HAL&);
			
			bool loadModule(std::string moduleName);
			
			void* getOutputFeature(std::string name, std::string featureName);
			void* getOutputFeature(OutputInterface * obj, std::string featureName);
			void* getInputFeature(std::string name, std::string featureName);
			void* getInputFeature(InputInterface * obj, std::string featureName);
			
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
