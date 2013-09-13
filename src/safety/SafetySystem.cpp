#include <eeros/safety/SafetySystem.hpp>

SafetySystem::SafetySystem() { }
SafetySystem::SafetySystem(const SafetySystem&) { }
SafetySystem& SafetySystem::operator=(const SafetySystem&) { }

void SafetySystem::setSafetyLevels(std::vector<SafetyLevel>& levels) {
	this->levels = levels;
}

// void SafetySystem::addCriticalOutput(std::string output) {
// 	
// }
// 
// void SafetySystem::addCriticalInput(std::string output, Event* event) {
// 	
// }

SafetyLevel& SafetySystem::level(uint32_t levelId) {
	int i = 0;
	
	while(i < levels.size()) {
		if(levels[i].getId() == levelId) return levels[i];
	}
	
	throw -1;
}

SafetyLevel& SafetySystem::operator[](unsigned levelId) {
	return level(levelId);
}

void SafetySystem::addEventToAllLevelsAbove(uint32_t level, uint32_t event, uint32_t nextLevel) {
	for(auto l : levels) {
		if(l.getId() >= level) l.addEvent(event, nextLevel);
	}
}

void SafetySystem::run() {
	// 1) Read inputs and throw event
	
	// 2) Execute level action
	
	// 3) Set outputs
}

SafetySystem& SafetySystem::instance() {
	static SafetySystem safetySystemInstance;
	return safetySystemInstance;
}