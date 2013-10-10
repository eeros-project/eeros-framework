#ifndef ORG_EEROS_SAFETY_INPUTACTION_HPP_
#define ORG_EEROS_SAFETY_INPUTACTION_HPP_

#include <stdint.h>
#include <eeros/hal/HAL.hpp>

class SafetyContext;

class InputAction {
public:
	InputAction(SystemInputInterface& inputInterface) : inputInterface(&inputInterface) { }
	virtual ~InputAction() { }
	virtual void check(SafetyContext* context) { }
	
	SystemInputInterface const* inputInterface;
};

#endif // ORG_EEROS_SAFETY_INPUTACTION_HPP_