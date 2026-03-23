#include <eeros/core/Fault.hpp>
#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/safety/InputAction.hpp>
#include <sstream>

namespace eeros {
namespace safety {

SafetyProperties::SafetyProperties() : entryLevel(nullptr) { count = 0; }

SafetyLevel* SafetyProperties::getEntryLevel() { return entryLevel; }

void SafetyProperties::setEntryLevel(SafetyLevel& entryLevel) {
  this->entryLevel = &entryLevel;
}

bool SafetyProperties::verify() {
  bool check = true;

  // Check in every level ...
  for (auto& l : levels) {
    // if the output action of every output is defined
    std::vector<hal::OutputInterface*> copy1 = criticalOutputs;
    for (auto& action : l->outputAction) {
      auto output = action->getOutput();
      std::vector<hal::OutputInterface*>::iterator it = copy1.begin();
      while (it != copy1.end()) {
        if (*it == output)
          it = copy1.erase(it);
        else
          ++it;
      }
    }
    if (!copy1.empty())
      throw Fault(
          "verification of safety properties failed, all critical outputs must "
          "be defined in level: " +
          l->getDescription());
    check = check && copy1.empty();

    // if the input action for every critical input is defined
    std::vector<hal::InputInterface*> copy2 = criticalInputs;
    for (auto& action : l->inputAction) {
      auto input = action->getInput();
      std::vector<hal::InputInterface*>::iterator it = copy2.begin();
      while (it != copy2.end()) {
        if (*it == input)
          it = copy2.erase(it);
        else
          ++it;
      }
    }
    if (!copy2.empty())
      throw Fault(
          "verification of safety properties failed, all critical inputs must "
          "be defined in level: " +
          l->getDescription());
    check = check && copy2.empty();
  }

  // Check entry level
  check = check && getEntryLevel() != nullptr;

  return check;
}

void SafetyProperties::addLevel(SafetyLevel& level) {
  level.id = count++;
  levels.push_back(&level);
}

void SafetyProperties::addEventToLevel(SafetyLevel& level, SafetyEvent event,
                                       SafetyLevel& nextLevel, EventType type) {
  level.addEvent(event, nextLevel, type);
}

void SafetyProperties::addEventToLevelAndAbove(SafetyLevel& level,
                                               SafetyEvent event,
                                               SafetyLevel& nextLevel,
                                               EventType type) {
  for (auto& lev : levels) {
    if (lev->id >= level.id) lev->addEvent(event, nextLevel, type);
  }
}

void SafetyProperties::addEventToLevelAndBelow(SafetyLevel& level,
                                               SafetyEvent event,
                                               SafetyLevel& nextLevel,
                                               EventType type) {
  for (auto& lev : levels) {
    if (lev->id <= level.id) lev->addEvent(event, nextLevel, type);
  }
}

void SafetyProperties::addEventToAllLevelsBetween(SafetyLevel& lowerLevel,
                                                  SafetyLevel& upperLevel,
                                                  SafetyEvent event,
                                                  SafetyLevel& nextLevel,
                                                  EventType type) {
  for (auto& lev : levels) {
    if (lev->id >= lowerLevel.id && lev->id <= upperLevel.id)
      lev->addEvent(event, nextLevel, type);
  }
}

};  // namespace safety
};  // namespace eeros
