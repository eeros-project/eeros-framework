#ifndef ORG_EEROS_SAFETY_INPUTACTION_HPP_
#define ORG_EEROS_SAFETY_INPUTACTION_HPP_

#include <stdint.h>
#include <eeros/hal/HAL.hpp>
//#include <eeros/safety/SafetySystem.hpp>

class InputAction {
public:
	virtual ~InputAction() { }
	virtual void check() { }
};

template <typename T>
class IgnoreInputAction : public InputAction {
public:
	IgnoreInputAction(SystemInput<T>& input) : input(input) { }
	virtual ~IgnoreInputAction() { }
	virtual void check() { }

private:
	SystemInput<T>& input;
};

template <typename T>
class CheckInputAction : public InputAction {
public:
	CheckInputAction(SystemInput<T>& input, T value, uint32_t event) : input(input), value(value), event(event) { }
	virtual ~CheckInputAction() { }
	virtual void check() {
//		if(input.get() != value) SafetySystem::instance().triggerEvent(event);
	}
	
private:
	SystemInput<T>& input;
	T value;
	uint32_t event;
};

template <typename T>
class CheckRangeInputAction : public InputAction {
public:
	CheckRangeInputAction(SystemInput<T>& input, T min, T max, uint32_t event) : input(input), min(min), max(max), event(event) { }
	virtual ~CheckRangeInputAction() { }
	virtual void check() {
		T value = input.get();
//		if (value < min || value > max) SafetySystem::instance().triggerEvent(event);
	}

private:
	SystemInput<T>& input;
	T min;
	T max;
	uint32_t event;
};

template <typename T>
IgnoreInputAction<T> ignore(SystemInput<T>& input) {
	return IgnoreInputAction<T>(input);
}

template <typename T>
CheckInputAction<T> check(SystemInput<T>& input, T value, uint32_t event) {
	return CheckInputAction<T>(input, value, event);
}

template <typename T>
CheckRangeInputAction<T> range(SystemInput<T>& input, T min, T max, uint32_t event) {
	return CheckRangeInputAction<T>(input, min, max, event);
}

#endif // ORG_EEROS_SAFETY_INPUTACTION_HPP_