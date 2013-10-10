#ifndef ORG_EEROS_SAFETY_SAFETYCONTEXT_HPP_
#define ORG_EEROS_SAFETY_SAFETYCONTEXT_HPP_

#include <eeros/safety/SafetyState.hpp>
#include <stdint.h>

class SafetySystem;

class SafetyContext {
	friend class SafetySystem;
public:
	void triggerEvent(int32_t event);
	
private:
	explicit SafetyContext(SafetyState& state);
	SafetyState& state;
};

#endif // ORG_EEROS_SAFETY_SAFETYCONTEXT_HPP_