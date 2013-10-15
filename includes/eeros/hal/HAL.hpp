#ifndef ORG_EEROS_HAL_HAL_HPP_
#define ORG_EEROS_HAL_HAL_HPP_

#include <string>

#include <eeros/hal/SystemInput.hpp>
#include <eeros/hal/SystemOutput.hpp>

class HAL {
public:
	SystemOutput<bool> getLogicSystemOutput(std::string name);
	SystemOutput<double> getRealSystemOutput(std::string name);
	SystemInput<bool> getLogicSystemInput(std::string name);
	SystemInput<double> getRealSystemInput(std::string name);
	
	static HAL& instance();
	
private:
	HAL();
	HAL(const HAL&);
	HAL& operator=(const HAL&);
};



#endif /* ORG_EEROS_HAL_HAL_HPP_ */