#ifndef ORG_EEROS_HAL_HAL_HPP_
#define ORG_EEROS_HAL_HAL_HPP_

#include <string>
#include <map>
#include <unordered_set>
#include <eeros/hal/Input.hpp>
#include <eeros/hal/Output.hpp>
#include <eeros/hal/JsonParser.hpp>

namespace eeros {
	namespace hal {
		
		class HAL {
		public:
			OutputInterface* getOutput(std::string name, bool exclusive = false);
			Output<bool>* getLogicOutput(std::string name, bool exclusive = false);
			Output<double>* getRealOutput(std::string name, bool exclusive = false);
			InputInterface* getInput(std::string name, bool exclusive = false);
			Input<bool>* getLogicInput(std::string name, bool exclusive = false);
			Input<double>* getRealInput(std::string name, bool exclusive = false);
			
			bool addInput(InputInterface* systemInput);
			bool addOutput(OutputInterface* systemOutput);
			
			bool readConfigFromFile(std::string file);
			
			static HAL& instance();
			
		private:
			HAL();
			HAL(const HAL&);
			HAL& operator=(const HAL&);
			
			bool loadModule(std::string moduleName);
			
			std::unordered_set<OutputInterface*> exclusiveReservedOutputs;
			std::unordered_set<InputInterface*> exclusiveReservedInputs;
			
			std::map<std::string, InputInterface*> inputs;
			std::map<std::string, OutputInterface*> outputs;
			
			std::map<std::string, void*> hwLibraries;
			JsonParser parser;
			
		};

	};
};

#endif /* ORG_EEROS_HAL_HAL_HPP_ */
