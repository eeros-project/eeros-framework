#include <eeros/safety/SafetySystem.hpp>
#include <eeros/core/EEROSException.hpp>

SafetySystem::SafetySystem() : privateContext(state) { }

SafetySystem::SafetySystem(const SafetySystem&) : privateContext(state) { }
SafetySystem& SafetySystem::operator=(const SafetySystem&) { }

SafetyLevel& SafetySystem::getLevel(uint32_t levelId) {
	return state.getLevel(levelId);
}

SafetyLevel& SafetySystem::operator[](unsigned levelId) {
	return getLevel(levelId);
}

void SafetySystem::defineSafetyLevels(std::vector<SafetyLevel> levels) {
	state.levels = levels;
}

void SafetySystem::setEntryLevel(uint32_t levelId) {
	if(!state.currentLevel) { // set only if currentLevel is 
		state.currentLevel = &(getLevel(levelId));
	}
	else throw EEROSException("currentLevel already defined"); // TODO define error number and send error message to logger
}

void SafetySystem::defineCriticalOutputs(std::vector<SystemOutputInterface*> outputs) {
	state.criticalOutputs = outputs;
}

void SafetySystem::defineCriticalInputs(std::vector<SystemInputInterface*> inputs) {
	state.criticalInputs = inputs;
}

void SafetySystem::addEventToLevel(uint32_t levelId, uint32_t event, uint32_t nextLevelId, EventType type) {
	SafetyLevel& level = getLevel(levelId);
	level.addEvent(event, nextLevelId, type);
}


void SafetySystem::addEventToLevelAndAbove(uint32_t levelId, uint32_t event, uint32_t nextLevelId, EventType type) {
	for(auto level : state.levels) {
		if(level.getId() >= levelId) level.addEvent(event, nextLevelId, type);
	}
}

void SafetySystem::addEventToLevelAndBelow(uint32_t levelId, uint32_t event, uint32_t nextLevelId, EventType type) {
	for(auto level : state.levels) {
		if(level.getId() <= levelId) level.addEvent(event, nextLevelId, type);
	}
}

void SafetySystem::addEventToAllLevelsBetween(uint32_t lowerLevelId, uint32_t upperLevelId, uint32_t event, uint32_t nextLevelId, EventType type) {
	for(auto level : state.levels) {
		if(level.getId() >= lowerLevelId && level.getId() <= upperLevelId) level.addEvent(event, nextLevelId, type);
	}
}

void SafetySystem::triggerEvent(uint32_t event) {
	if(state.currentLevel) {
		uint32_t nextLevelId = state.currentLevel->getLevelIdForEvent(event, false);
		if(nextLevelId != kInvalidLevel) {
			SafetyLevel* nextLevel = &(getLevel(nextLevelId));
			state.currentLevel = nextLevel; // TODO make atomic
		}
	}
	else {
		throw EEROSException("currentLevel not defiend"); // TODO define error number and send error message to logger
	}
}

void SafetySystem::run() {
	// 1) Make local copy of currentLevel
	SafetyLevel* level = state.currentLevel;
	
	// 2) Read inputs
	for(auto ia : level->inputAction) {
		ia.check();
	}
	
	// 3) Execute level action
	if(level->action != nullptr) {
		level->action(&privateContext);
	}
		
	// 4) Set outputs
	for(auto oa : level->outputAction) {
		oa.set();
	}
}

SafetySystem& SafetySystem::instance() {
	static SafetySystem safetySystemInstance;
	return safetySystemInstance;
}