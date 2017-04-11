#ifndef ORG_EEROS_HAL_HAL_HPP_
#define ORG_EEROS_HAL_HAL_HPP_

#include <string>
#include <map>
#include <unordered_set>
#include <eeros/hal/Input.hpp>
#include <eeros/hal/Output.hpp>
#include <eeros/hal/ScalableOutput.hpp>
#include <eeros/hal/ScalableInput.hpp>
#include <eeros/hal/JsonParser.hpp>
#include <eeros/core/Fault.hpp>


namespace eeros {
	namespace hal {
		
		class HAL {
		public:
			OutputInterface* getOutput(std::string name, bool exclusive = true);
			Output<bool>* getLogicOutput(std::string name, bool exclusive = true);
			ScalableOutput<double>* getScalableOutput(std::string name, bool exclusive = true);
			InputInterface* getInput(std::string name, bool exclusive = true);
			Input<bool>* getLogicInput(std::string name, bool exclusive = true);
			ScalableInput<double>* getScalableInput(std::string name, bool exclusive = true);
			void releaseInput(std::string name);
			void releaseOutput(std::string name);
			
			bool addInput(InputInterface* systemInput);
			bool addOutput(OutputInterface* systemOutput);
			
			bool readConfigFromFile(std::string file);
			bool readConfigFromFile(int* argc, char** argv);
			
			static HAL& instance();
						
			template<typename ... ArgTypesOut>
			void callOutputFeature(OutputInterface *obj, std::string featureName, ArgTypesOut... args){
				
				void (*featureFunction)(OutputInterface*, ArgTypesOut...) = reinterpret_cast<void(*)(OutputInterface*, ArgTypesOut...)>(getOutputFeature(obj, featureName));
				if(featureFunction == nullptr){
					throw Fault("could not find method in dynamic library: " + featureName);
				}
				featureFunction(obj, args...);
			}
			
			template<typename ... ArgTypesIn>
			void callInputFeature(InputInterface *obj, std::string featureName, ArgTypesIn... args){
				
				void (*featureFunction)(InputInterface*, ArgTypesIn...) = reinterpret_cast<void(*)(InputInterface*, ArgTypesIn...)>(getInputFeature(obj, featureName));
				if(featureFunction == nullptr){
					throw Fault("could not find method in dynamic library: " + featureName);
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
			
			logger::Logger log;
			
		};

	};
};

#endif /* ORG_EEROS_HAL_HAL_HPP_ */
