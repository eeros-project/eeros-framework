#ifndef ORG_EEROS_HAL_HAL_HPP_
#define ORG_EEROS_HAL_HAL_HPP_

#include <string>
#include <map>
#include <unordered_set>

#include <eeros/hal/SystemInput.hpp>
#include <eeros/hal/SystemOutput.hpp>
#include "ComediDevice.hpp"

namespace eeros {
	namespace hal {
		
		class HAL {
		public:
			SystemOutputInterface* getSystemOutput(std::string name, bool exclusive = false);
			SystemOutput<bool>* getLogicSystemOutput(std::string name, bool exclusive = false);
			SystemOutput<double>* getRealSystemOutput(std::string name, bool exclusive = false);
			SystemInputInterface* getSystemInput(std::string name, bool exclusive = false);
			SystemInput<bool>* getLogicSystemInput(std::string name, bool exclusive = false);
			SystemInput<double>* getRealSystemInput(std::string name, bool exclusive = false);
			
			bool addSystemInput(SystemInputInterface* systemInput);
			bool addSystemOutput(SystemOutputInterface* systemOutput);
			
			bool readConfigFromFile(std::string file);
			
			static HAL& instance();
			
		private:
			HAL();
			HAL(const HAL&);
			HAL& operator=(const HAL&);
			
			bool loadModule(std::string moduleName);
			
			std::unordered_set<SystemOutputInterface*> exclusiveReservedOutputs;
			std::unordered_set<SystemInputInterface*> exclusiveReservedInputs;
			
			std::map<std::string, SystemInputInterface*> inputs;
			std::map<std::string, SystemOutputInterface*> outputs;
		};

	};
};

#endif /* ORG_EEROS_HAL_HAL_HPP_ */