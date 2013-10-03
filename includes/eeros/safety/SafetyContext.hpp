#ifndef ORG_EEROS_SAFETY_SAFETYCONTEXT_HPP_
#define ORG_EEROS_SAFETY_SAFETYCONTEXT_HPP_

#include <eeros/safety/SafetyState.hpp>
#include <eeros/types.hpp>

class SafetySystem;

class SafetyContext {
	friend class SafetySystem;
public:
	void triggerEvent(uint32_t event);
	
private:
//	SafetyContext();
	explicit SafetyContext(SafetyState& state);
	SafetyState& state;
};

#endif // ORG_EEROS_SAFETY_SAFETYCONTEXT_HPP_