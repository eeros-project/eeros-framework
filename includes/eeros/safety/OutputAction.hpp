#ifndef ORG_EEROS_SAFETY_OUTPUTACTION_HPP_
#define ORG_EEROS_SAFETY_OUTPUTACTION_HPP_

#include <stdint.h>
#include <eeros/hal/HAL.hpp>

////////////////////////////// TODO split .hpp/.cpp //////////////////////////////

class OutputAction {
public:
	virtual ~OutputAction(void) { }
	virtual void set() { };
};

template < typename T >
class LeaveOutputAction : public OutputAction {
public:
	LeaveOutputAction(SystemOutput<T>& output) : output(output) { }
	virtual ~LeaveOutputAction(void) { }
	virtual void set() {  }
private:
	SystemOutput<T>& output;
};

template < typename T >
class SetOutputAction : public OutputAction {
public:
	SetOutputAction(SystemOutput<T>& output, T value) : output(output), value(value) { }
	virtual ~SetOutputAction(void) { }
	virtual void set() { output.set(value); }
private:
	SystemOutput<T>& output;
	T value;
};

#endif // ORG_EEROS_SAFETY_OUTPUTACTION_HPP_
