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
	// TODO
}

void SafetySystem::run() {
	
}

SafetySystem& SafetySystem::instance() {
	
}