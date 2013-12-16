#include <eeros/safety/SafetyContext.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/core/EEROSException.hpp>
#include <eeros/safety/SafetySystem.hpp>

using namespace eeros::safety;

SafetyContext::SafetyContext() { }


void SafetyContext::triggerEvent(int32_t event) {
	// Get Safety System instance
	SafetySystem& safetySys = SafetySystem::instance();
	
	// Trigger event in private context
	safetySys.triggerEvent(event, this);
}