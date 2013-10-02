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
		if((it->second.second != kPrivateEvent) || privateEventOk) return it->second.first;
	}
	return kInvalidLevel;
}

void SafetyLevel::addEvent(uint32_t event, uint32_t nextLevelId, EventType type) {
	transitions.insert(std::make_pair(event, std::make_pair(nextLevelId, type)));
}

void SafetyLevel::setLevelAction(std::function<void (SafetyContext*)> action) {
	this->action = action;
}


void SafetyLevel::setInputAction(InputAction action) {
	inputAction.push_back(action);
}

void SafetyLevel::setInputActions(std::vector<InputAction> actionList) {
	inputAction = actionList;
}

void SafetyLevel::setOutputAction(OutputAction action) {
	outputAction.push_back(action);
}

void SafetyLevel::setOutputActions(std::vector<OutputAction> actionList) {
	outputAction = actionList;
}