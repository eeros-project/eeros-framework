#include <eeros/safety/SafetyContext.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/core/EEROSException.hpp>

using namespace eeros::safety;

SafetyContext::SafetyContext(SafetyState& state) : state(state) { }

void SafetyContext::triggerEvent(int32_t event) {
	if(state.currentLevel) {
		int32_t nextLevelId = state.currentLevel->getLevelIdForEvent(event, true);
		if(nextLevelId != kInvalidLevel) {
			SafetyLevel* nextLevel = &(state.getLevel(nextLevelId));
			if(nextLevel != nullptr) {
				state.currentLevel = nextLevel; // TODO make atomic
			}
			else {
				throw EEROSException("unknown safety level"); // TODO define error number and send error message to logger
			}
			
		}
		else {
			// TODO send msg to logger
		}
	}
	else {
		throw EEROSException("current level not defined"); // TODO define error number and send error message to logger
	}
}
