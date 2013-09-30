#ifndef ORG_EEROS_HAL_HAL_HPP_
#define ORG_EEROS_HAL_HAL_HPP_

#include <string>

/////////////////////////////// TODO move this to separate file ///////////////////////////////
template <typename T>
class SystemInput {
public:
	virtual ~SystemInput(void) { }
	virtual T& get(void) { return value; }
protected:
	T value;
};

template <typename T>
class SystemOutput : public SystemInput<T> {
public:
	virtual ~SystemOutput(void) { }
	virtual T& get(void) { return this->value; }
	virtual void set(T value) { this->value = value; }
	virtual SystemInput<T>& getInput(void) { return static_cast<SystemInput<T>&>(*this); }
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

