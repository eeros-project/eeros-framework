#include <eeros/safety/SafetyContext.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/safety/SafetySystem.hpp>

using namespace eeros::safety;

SafetyContext::SafetyContext(SafetySystem* parent) : parent(parent) { }


void SafetyContext::triggerEvent(SafetyEvent event) {	
	// Trigger event in private context
	parent->triggerEvent(event, this);
}
