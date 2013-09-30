#include <eeros/safety/SafetyLevel.hpp>

SafetyLevel::SafetyLevel(uint32_t id, std::string description) : id(id), description(description) {
	// nothing to do...
}

SafetyLevel::~SafetyLevel() {
	// nothing to do...
}


uint32_t SafetyLevel::getId() {
	return id;
}

uint32_t SafetyLevel::getLevelIdForEvent(uint32_t event, bool privateEventOk) {
	auto it = transitions.find(event);
	if(it != transitions.end()) {
	// TODO
		return it->second;
	}
	return kInvalidLevel;
}

void SafetyLevel::addEvent(uint32_t event, uint32_t nextLevelId, EventType type) {
	//transitions.insert(std::make_pair(event, nextLevel));
	//transitions[event] = nextLevelId
	// TODO
}

void SafetyLevel::setLevelAction(std::function<void (SafetyContext*)> action) {
	this->action = action;
}


void SafetyLevel::setInputAction(InputAction action) {
	// TODO
}

void SafetyLevel::setInputActions(std::vector<InputAction> actionList) {
	// TODO
}

void SafetyLevel::setOutputAction(OutputAction action) {
	// TODO
}

void SafetyLevel::setOutputActions(std::vector<OutputAction> actionList) {
	// TODO
}