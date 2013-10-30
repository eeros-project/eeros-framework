#include <eeros/safety/SafetyState.hpp>
#include <eeros/core/EEROSException.hpp>

using namespace eeros::safety;

SafetyState::SafetyState() : currentLevel(0) { }

SafetyLevel& SafetyState::getLevel(int32_t levelId) {
	for(auto& level : levels) {
		if(level.getId() == levelId) return level;
	}
	throw EEROSException("level not defined"); // TODO define error number and send error message to logger 
}