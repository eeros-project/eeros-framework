#include <eeros/safety/SafetyContext.hpp>
#include <eeros/safety/SafetyLevel.hpp>

SafetyContext::SafetyContext(SafetyState& state) : state(state) { }

//SafetyContext::SafetyContext() { }

void SafetyContext::triggerEvent(uint32_t event) {
	if(state.currentLevel) {
		uint32_t nextLevelId = state.currentLevel->getLevelIdForEvent(event, true);
		if(nextLevelId != kInvalidLevel) {
			SafetyLevel* nextLevel = &(state.getLevel(nextLevelId));
			state.currentLevel = nextLevel; // TODO make atomic
		}
	}
	else {
		throw -1; // TODO define error number and send error message to logger
	}
}
	
