#include <eeros/safety/SafetyLevel.hpp>

SafetyLevel::SafetyLevel(uint32_t id, std::string description, std::map<std::string, bool> outputStates) : id(id), description(description), outputs(outputStates) {
	// nothing to do
}

uint32_t SafetyLevel::getId() {
	return id;
}

void SafetyLevel::addEvent(uint32_t event, uint32_t nextLevel) {
	transitions.insert(std::make_pair(event, nextLevel));
}

void SafetyLevel::setExitEvent(uint32_t event) {
	
}

void SafetyLevel::setLevelAction(std::function<void (void)> action) {
	this->action = action;
}

SafetyLevel::~SafetyLevel() {
	
}
