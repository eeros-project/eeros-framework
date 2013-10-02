#ifndef ORG_EEROS_SAFETY_OUTPUTACTION_HPP_
#define ORG_EEROS_SAFETY_OUTPUTACTION_HPP_

#include <stdint.h>
#include <eeros/hal/HAL.hpp>

class OutputAction {
public:
	virtual ~OutputAction() { }
	virtual void set() { }
};

template < typename T >
class LeaveOutputAction : public OutputAction {
public:
	LeaveOutputAction(SystemOutput<T>& output) { }
	virtual ~LeaveOutputAction() { }
	virtual void set() { }
private:
	SystemOutput<T>& output;
};

template < typename T >
class SetOutputAction : public OutputAction {
public:
	SetOutputAction(SystemOutput<T>& output, T value) : output(output), value(value) { }
	virtual ~SetOutputAction() { }
	virtual void set() { 
		output.set(value);
	}
private:
	SystemOutput<T>& output;
	T value;
};

template <typename T>
SetOutputAction<T> set(SystemOutput<T>& output, T value) {
	return SetOutputAction<T>(output, value);
}

template <typename T>
LeaveOutputAction<T> leave(SystemOutput<T>& output) {
	return LeaveOutputAction<T>(output);
}

#endif // ORG_EEROS_SAFETY_OUTPUTACTION_HPP_
