#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/safety/InputAction.hpp>

namespace eeros {
namespace safety {

SafetyEvent::SafetyEvent(std::string description) : description(description) {
  static int count = 0;
  id = count++;
}

std::string SafetyEvent::getDescription() {
  return description;
}

SafetyLevel::SafetyLevel(std::string description) : description(description), log(logger::Logger::getLogger('S')) {
  // number the levels when adding them to the safety system
}

bool SafetyLevel::operator<(const SafetyLevel& level) {
  return this->id < level.id;
}

bool SafetyLevel::operator<=(const SafetyLevel& level) {
  return this->id <= level.id;
}

bool SafetyLevel::operator>(const SafetyLevel& level) {
  return this->id > level.id;
}

bool SafetyLevel::operator>=(const SafetyLevel& level) {
  return this->id >= level.id;
}

bool SafetyLevel::operator==(const SafetyLevel& level) {
  return this->id == level.id;
}

bool SafetyLevel::operator!=(const SafetyLevel& level) {
  return this->id != level.id;
}

std::string SafetyLevel::getDescription() {
  return description;
}

uint32_t SafetyLevel::getLevelId() {
  return id;
}

uint32_t SafetyLevel::getNofActivations() {
  return nofActivations;
}

SafetyLevel* SafetyLevel::getDestLevelForEvent(SafetyEvent event, bool privateEventOk) {
  auto it = transitions.find(event.id);
  if(it != transitions.end()) {
    if((it->second.second != kPrivateEvent) || privateEventOk) return it->second.first;
    else log.error() << "triggering private event \'" << event << "\' from nonprivate context";
  }
  return nullptr;
}

void SafetyLevel::addEvent(SafetyEvent event, SafetyLevel& nextLevel, EventType type) {
  transitions.insert(std::make_pair(event.id, std::make_pair(&nextLevel, type)));
}

void SafetyLevel::setLevelAction(std::function<void (SafetyContext*)> action) {
  this->action = action;
}

void SafetyLevel::setEntryAction(std::function<void (SafetyContext*)> action) {
  this->onEntry = action;
}

void SafetyLevel::setExitAction(std::function<void ()> action) {
  this->onExit = action;
}

void SafetyLevel::setInputAction(InputAction* action) {
  inputAction.push_back(action);
}

void SafetyLevel::setInputActions(std::vector<InputAction*> actionList) {
  inputAction = actionList;
}

void SafetyLevel::setOutputAction(OutputAction* action) {
  outputAction.push_back(action);
}

void SafetyLevel::setOutputActions(std::vector<OutputAction*> actionList) {
  outputAction = actionList;
}

/********** Print functions **********/
std::ostream& operator<<(std::ostream& os, eeros::safety::SafetyEvent& event) {
  os << event.getDescription();
  return os;
}

std::ostream& operator<<(std::ostream& os, eeros::safety::SafetyEvent* event) {
  os << event->getDescription();
  return os;
}

std::ostream& operator<<(std::ostream& os, eeros::safety::SafetyLevel& level) {
  os << level.getDescription();
  return os;
}

std::ostream& operator<<(std::ostream& os, eeros::safety::SafetyLevel* level) {
  os << level->getDescription();
  return os;
};

}
}
