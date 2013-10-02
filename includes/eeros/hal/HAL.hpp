#ifndef ORG_EEROS_HAL_HAL_HPP_
#define ORG_EEROS_HAL_HAL_HPP_

#include <string>

/////////////////////////////// TODO move this to separate file ///////////////////////////////

class SystemInputInterface { };
class SystemOutputInterface { };


template <typename T>
class SystemInput : public SystemInputInterface {
public:
	explicit SystemInput(T& value) : value(value) { }
	inline T& get(void) { return value; }
private:
	T& value;
};

template <typename T>
class SystemOutput : public SystemOutputInterface {
public:
	explicit SystemOutput(T& value) : value(value) { }
	inline T& get(void) { return value; }
	inline void set(T value) { this->value = value; }
	inline SystemInput<T> getInput(void) { return SystemInput<T>(value); }
private:
	T& value;
};
///////////////////////////////////////////////////////////////////////////////////////////////

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

