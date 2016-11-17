#ifndef ORG_EEROS_HAL_HAL_HPP_
#define ORG_EEROS_HAL_HAL_HPP_

#include <string>
#include <map>
#include <unordered_set>
#include <eeros/hal/PeripheralInput.hpp>
#include <eeros/hal/PeripheralOutput.hpp>
#include <eeros/hal/JsonParser.hpp>

namespace eeros {
	namespace hal {
		
		class HAL {
		public:
			PeripheralOutputInterface* getPeripheralOutput(std::string name, bool exclusive = false);
			PeripheralOutput<bool>* getLogicPeripheralOutput(std::string name, bool exclusive = false);
			PeripheralOutput<double>* getRealPeripheralOutput(std::string name, bool exclusive = false);
			PeripheralInputInterface* getPeripheralInput(std::string name, bool exclusive = false);
			PeripheralInput<bool>* getLogicPeripheralInput(std::string name, bool exclusive = false);
			PeripheralInput<double>* getRealPeripheralInput(std::string name, bool exclusive = false);
			
			bool addPeripheralInput(PeripheralInputInterface* systemInput);
			bool addPeripheralOutput(PeripheralOutputInterface* systemOutput);
			
			bool readConfigFromFile(std::string file);
			
			static HAL& instance();
			
		private:
			HAL();
			HAL(const HAL&);
			HAL& operator=(const HAL&);
			
			bool loadModule(std::string moduleName);
			
			std::unordered_set<PeripheralOutputInterface*> exclusiveReservedOutputs;
			std::unordered_set<PeripheralInputInterface*> exclusiveReservedInputs;
			
			std::map<std::string, PeripheralInputInterface*> inputs;
			std::map<std::string, PeripheralOutputInterface*> outputs;
			
			std::map<std::string, void*> hwLibraries;
			JsonParser parser;
			
		};

	};
};

#endif /* ORG_EEROS_HAL_HAL_HPP_ */
