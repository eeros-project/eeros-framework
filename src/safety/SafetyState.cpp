#include <eeros/safety/SafetyState.hpp>

SafetyState::SafetyState() : currentLevel(0) { }

SafetyLevel& SafetyState::getLevel(uint32_t levelId) {
	for(auto& level : levels) {
		if(level.getId() == levelId) return level;
	}
	throw -1; // TODO define error number and send error message to logger 
}