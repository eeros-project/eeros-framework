#ifndef ORG_EEROS_HAL_SYSTEMOUTPUT_HPP_
#define ORG_EEROS_HAL_SYSTEMOUTPUT_HPP_

#include <eeros/hal/SystemInput.hpp>

class SystemOutputInterface { };

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

#endif /* ORG_EEROS_HAL_SYSTEMOUTPUT_HPP_ */