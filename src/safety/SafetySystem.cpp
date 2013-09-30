#include <eeros/safety/SafetySystem.hpp>

SafetySystem::SafetySystem() : currentLevel(0) {
	privateContext = new SafetyContext();
}

SafetySystem::SafetySystem(const SafetySystem&) { }
SafetySystem& SafetySystem::operator=(const SafetySystem&) { }

SafetyLevel& SafetySystem::getLevel(uint32_t levelId) {
	// TODO fix warning: "reference to local variable ‘level’ returned"
	for(auto level : levels) {
		if(level.getId() == levelId) return level;
	}
	throw -1; // TODO define error number and send error message to logger
}

SafetyLevel& SafetySystem::operator[](unsigned levelId) {
	return getLevel(levelId);
}

void SafetySystem::defineSafetyLevels(std::vector<SafetyLevel> levels) {
	this->levels = levels;
}

void SafetySystem::setEntryLevel(uint32_t levelId) {
	if(!currentLevel) { // set only if currentLevel is 
		currentLevel = &(getLevel(levelId));
	}
	throw -1; // TODO define error number and send error message to logger
}

// TODO
// void SafetySystem::defineCriticalOutputs(std::vector<CriticalOutput> outputs) {
// 	criticalOutputs = outputs;
// }
// 
// void SafetySystem::defineCriticalInputs(std::vector<CriticalInput> inputs) {
// 	criticalInputs = inputs;
// }

void SafetySystem::addEventToLevel(uint32_t levelId, uint32_t event, uint32_t nextLevelId, EventType type) {
	SafetyLevel& level = getLevel(levelId);
	level.addEvent(event, nextLevelId, type);
}


void SafetySystem::addEventToLevelAndAbove(uint32_t levelId, uint32_t event, uint32_t nextLevelId, EventType type) {
	for(auto level : levels) {
		if(level.getId() >= levelId) level.addEvent(event, nextLevelId, type);
	}
}

void SafetySystem::addEventToLevelAndBelow(uint32_t levelId, uint32_t event, uint32_t nextLevelId, EventType type) {
	for(auto level : levels) {
		if(level.getId() <= levelId) level.addEvent(event, nextLevelId, type);
	}
}

void SafetySystem::addEventToAllLevelsBetween(uint32_t lowerLevelId, uint32_t upperLevelId, uint32_t event, uint32_t nextLevelId, EventType type) {
	for(auto level : levels) {
		if(level.getId() >= lowerLevelId && level.getId() <= upperLevelId) level.addEvent(event, nextLevelId, type);
	}
}

void SafetySystem::triggerEvent(uint32_t event, SafetyContext* context) {
	if(currentLevel) {
		uint32_t nextLevelId = currentLevel->getLevelIdForEvent(event, context == privateContext);
		if(nextLevelId != kInvalidLevel) {
			SafetyLevel* nextLevel = &(getLevel(nextLevelId));
			currentLevel = nextLevel;
		}
	}
	else {
		throw -1; // TODO define error number and send error message to logger
	}
}

void SafetySystem::run() {
	// 1) Make local copy of currentLevel
	SafetyLevel* level = this->currentLevel;
	
	// 2) Read inputs
	for(auto ia : level->inputAction) {
		ia.check();
	}
	
	// 3) Execute level action
	currentLevel->action(privateContext);
	
	// 4) Set outputs
	for(auto oa : level->outputAction) {
		oa.set();
	}
}

SafetySystem& SafetySystem::instance() {
	static SafetySystem safetySystemInstance;
	return safetySystemInstance;
}