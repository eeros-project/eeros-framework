#include <eeros/safety/SafetySystem.hpp>
#include <eeros/core/EEROSException.hpp>

using namespace eeros::safety;
using namespace eeros::hal;

SafetySystem::SafetySystem() : privateContext(state), wdOutput(nullptr), wdOutputState(false) { }

SafetySystem::SafetySystem(const SafetySystem&) : privateContext(state), wdOutput(nullptr) { }
SafetySystem& SafetySystem::operator=(const SafetySystem&) { }

SafetyLevel& SafetySystem::getCurrentLevel(void) {
	if(state.currentLevel) {
		return *state.currentLevel;
	}
	else {
		throw EEROSException("currentLevel not defiend"); // TODO define error number and send error message to logger
	}
}

SafetyLevel& SafetySystem::getLevel(int32_t levelId) {
	return state.getLevel(levelId);
}

SafetyLevel& SafetySystem::operator[](unsigned levelId) {
	return getLevel(levelId);
}

void SafetySystem::defineSafetyLevels(std::vector<SafetyLevel> levels) {
	state.levels = levels;
}

void SafetySystem::setEntryLevel(int32_t levelId) {
	if(!state.currentLevel) { // set only if currentLevel is not null
		state.currentLevel = &(getLevel(levelId));
	}
	else {
		throw EEROSException("currentLevel already defined"); // TODO define error number and send error message to logger
	}
}

void SafetySystem::defineCriticalOutputs(std::vector<SystemOutputInterface*> outputs) {
	state.criticalOutputs = outputs;
}

void SafetySystem::defineCriticalInputs(std::vector<SystemInputInterface*> inputs) {
	state.criticalInputs = inputs;
}

void SafetySystem::addCriticalInput(SystemInputInterface& input) {
	state.criticalInputs.push_back(&input);
}

void SafetySystem::addCriticalOutput(SystemOutputInterface& output) {
	state.criticalOutputs.push_back(&output);
}

void SafetySystem::addEventToLevel(int32_t levelId, uint32_t event, int32_t nextLevelId, EventType type) {
	SafetyLevel& level = getLevel(levelId);
	level.addEvent(event, nextLevelId, type);
}

void SafetySystem::addEventToLevelAndAbove(int32_t levelId, uint32_t event, int32_t nextLevelId, EventType type) {
	for(auto level : state.levels) {
		if(level.getId() >= levelId) level.addEvent(event, nextLevelId, type);
	}
}

void SafetySystem::addEventToLevelAndBelow(int32_t levelId, uint32_t event, int32_t nextLevelId, EventType type) {
	for(auto level : state.levels) {
		if(level.getId() <= levelId) level.addEvent(event, nextLevelId, type);
	}
}

void SafetySystem::addEventToAllLevelsBetween(int32_t lowerLevelId, int32_t upperLevelId, uint32_t event, int32_t nextLevelId, EventType type) {
	for(auto level : state.levels) {
		if(level.getId() >= lowerLevelId && level.getId() <= upperLevelId) level.addEvent(event, nextLevelId, type);
	}
}

void SafetySystem::triggerEvent(uint32_t event) {
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

void SafetySystem::run() {
	// 0) Check if all safety critical I/Os are defined in all levels
	static bool first = true;
	if(first) {
		first = false;
		if (state.currentLevel == nullptr) {
			throw EEROSException("current level not defiend! May forget to set entry level?");
		}
		for (auto level : state.levels) {
			// TODO check if all input/output actions are defined
		}
	}
	
	// 1) Make local copy of currentLevel
	SafetyLevel* level = state.currentLevel;
	
	// 2) Read inputs
	for(auto ia : level->inputAction) {
		if(ia != nullptr) {
			ia->check(&privateContext);
		}
	}
	
	// 3) Execute level action
	if(level->action != nullptr) {
		level->action(&privateContext);
	}
		
	// 4) Set outputs
	for(auto oa : level->outputAction) {
		if(oa != nullptr) {
			oa->set();
		}
	}
}

SafetySystem& SafetySystem::instance() {
	static SafetySystem safetySystemInstance;
	return safetySystemInstance;
}