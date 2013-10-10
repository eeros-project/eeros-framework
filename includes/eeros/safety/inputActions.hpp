#ifndef ORG_EEROS_SAFETY_INPUTACTIONS_HPP_
#define ORG_EEROS_SAFETY_INPUTACTIONS_HPP_

#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/SafetyContext.hpp>

template <typename T>
class IgnoreInputAction : public InputAction {
public:
	IgnoreInputAction(SystemInput<T>& input) : InputAction(input), input(input) { }
	virtual ~IgnoreInputAction() { }
	virtual void check(SafetyContext* context) { }

private:
	SystemInput<T>& input;
};

template <typename T>
class CheckInputAction : public InputAction {
public:
	CheckInputAction(SystemInput<T>& input, T value, uint32_t event) : InputAction(input), input(input), value(value), event(event) { }
	virtual ~CheckInputAction() { }
	virtual void check(SafetyContext* context) {
		if(input.get() != value) context->triggerEvent(event);
	}
	
private:
	SystemInput<T>& input;
	T value;
	uint32_t event;
};

template <typename T>
class CheckRangeInputAction : public InputAction {
public:
	CheckRangeInputAction(SystemInput<T>& input, T min, T max, uint32_t event) : InputAction(input), input(input), min(min), max(max), event(event) { }
	virtual ~CheckRangeInputAction() { }
	virtual void check(SafetyContext* context) {
		T value = input.get();
		if (value < min || value > max) context->triggerEvent(event);
	}

private:
	SystemInput<T>& input;
	T min;
	T max;
	uint32_t event;
};

template <typename T>
IgnoreInputAction<T>* ignore(SystemInput<T>& input) {
	return new IgnoreInputAction<T>(input);
}

template <typename T>
CheckInputAction<T>* check(SystemInput<T>& input, T value, uint32_t event) {
	return new CheckInputAction<T>(input, value, event);
}

template <typename T>
CheckRangeInputAction<T>* range(SystemInput<T>& input, T min, T max, uint32_t event) {
	return new CheckRangeInputAction<T>(input, min, max, event);
} 

#endif // ORG_EEROS_SAFETY_INPUTACTIONS_HPP_