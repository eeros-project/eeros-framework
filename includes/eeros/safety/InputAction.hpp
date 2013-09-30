#ifndef ORG_EEROS_SAFETY_INPUTACTION_HPP_
#define ORG_EEROS_SAFETY_INPUTACTION_HPP_

#include <stdint.h>
#include <eeros/hal/HAL.hpp>

////////////////////////////// TODO split .hpp/.cpp //////////////////////////////

class InputAction {
public:
	virtual ~InputAction(void) { }
	virtual void check() { };
};

template < typename T >
class IgnoreInputAction : public InputAction {
public:
	IgnoreInputAction(SystemInput<T>& input) : input(input) { }
	virtual ~IgnoreInputAction(void) { }
	virtual void check() { }
private:
	SystemInput<T>& input;
};

template < typename T >
class CheckInputAction : public InputAction {
public:
	CheckInputAction(SystemInput<T>& input, T value, uint32_t event) : input(input), value(value), e(e) { }
	virtual ~CheckInputAction(void) { }
	virtual void check() {
		// if (input.get() != value) context.trigger(e);
		// TODO
	}
private:
	SystemInput<T>& input;
	T value;
	uint32_t e;
};

template < typename T >
class CheckRangeInputAction : public InputAction {
public:
	CheckRangeInputAction(SystemInput<T>& input, T min, T max, uint32_t event) : input(input), min(min), max(max), e(e) { }
	virtual ~CheckRangeInputAction(void) { }
	virtual void check() {
		T value = input.get();
		// if (value < min || value > max) SafetySystem::.trigger(e);
		// TODO
}

private:
	SystemInput<T>& input;
	T min;
	T max;
	uint32_t e;
};

#endif // ORG_EEROS_SAFETY_INPUTACTION_HPP_
