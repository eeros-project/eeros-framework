#ifndef ORG_EEROS_HAL_HAL_HPP_
#define ORG_EEROS_HAL_HAL_HPP_

#include <string>
#include <map>

#include <eeros/hal/SystemInput.hpp>
#include <eeros/hal/SystemOutput.hpp>
#include "ComediDevice.hpp"

class HAL {
public:
	SystemOutput<bool>& getLogicSystemOutput(std::string name);
	SystemOutput<double>& getRealSystemOutput(std::string name);
	SystemInput<bool>& getLogicSystemInput(std::string name);
	SystemInput<double>& getRealSystemInput(std::string name);
	
	void addSystemInput(SystemInputInterface* systemInput);
	void addSystemOutput(SystemOutputInterface* systemOutput);
	
	static HAL& instance();
	
private:
	HAL();
	HAL(const HAL&);
	HAL& operator=(const HAL&);

	std::map<std::string, ComediDevice> devices;
	
	std::map<std::string, SystemInputInterface*> inputs;
	std::map<std::string, SystemOutputInterface*> outputs;
};

#endif /* ORG_EEROS_HAL_HAL_HPP_ */