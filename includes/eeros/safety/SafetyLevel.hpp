#ifndef ORG_EEROS_SAFETY_SAFETYLEVEL_HPP_
#define ORG_EEROS_SAFETY_SAFETYLEVEL_HPP_

#include <stdint.h>
#include <string>
#include <vector>
#include <sstream>
#include <map>
#include <functional>
#include <eeros/safety/OutputAction.hpp>

namespace eeros {
namespace safety {

class SafetyContext;
class SafetySystem;
class InputAction;

enum EventType { kPrivateEvent, kPublicEvent };

/** \brief Safety event.
 *
 * Safety events serve to switch between different safety levels. An event must be registered for a given
 * source level and switches to a predefined destination level. Events can be private or public. Private
 * means that it can be caused solely by the safety system itself. Public means that it could be triggered
 * by the control system or the sequencer as well.
 */
class SafetyEvent {
  friend class SafetyLevel;

 public:
  /**
   * Creates a safety event.
   *
   * @param description The name of this event
   */
  SafetyEvent(std::string description);

  /**
   * Getter function for the name of this event.
   *
   * @return the name of this event
   */
  std::string getDescription();

 private:
  std::string description;
  uint32_t id;
};

/** \brief Safety level.
 *
 * The safety system comprises several safety levels.
 */
class SafetyLevel {
  friend class SafetySystem;
  friend class SafetyProperties;

 public:
  /**
   * Creates a safety level.
   *
   * @param description The name of this level
   */
  SafetyLevel(std::string description);

  /**
   * Getter function for the name of this level.
   *
   * @return the name of this level
   */
  std::string getDescription();

  /**
   * Getter function for the current safety level id. When safety levels are added to
   * the safety system, they are numbered with increasing numbers. This is their id.
   *
   * @return the current safety level id
   */
  uint32_t getLevelId();

  /**
   * When the safety system runs, it executes its code given by the current level. Each time
   * the current level is executed the variable nofActivations is increased by one.
   * This function returns the value of this variable.
   *
   * @return the number of activations this level has been executed
   */
  uint32_t getNofActivations();

  /**
   * Every safety event has exactly one destination level to which the safety system will
   * switch when this safety event occurs. Returns this destination level. Logs an error if the
   * event is private but event was triggered from outside of the safety system.
   *
   * @param event safety event
   * @param privateEventOk true: is a private event, false: is a public event
   *
   * @return the destination level of this event,
   *         nullptr: if no event was registered for this level or registered event is private
   */
  SafetyLevel* getDestLevelForEvent(SafetyEvent event, bool privateEventOk = false);

  /**
   * Adds an event to a safety level. Every safety event has exactly one destination level
   * to which the safety system will switch when this safety event occurs.
   *
   * @param event safety event
   * @param nextLevel registered event will switch to this level
   * @param type true: is a private event, false: is a public event
   */
  void addEvent(SafetyEvent event, SafetyLevel& nextLevel, EventType type = kPrivateEvent);

  /**
   * Critical inputs must be checked for every safety level. Adds an input action
   * to the list of actions. Such an action is a function (check, ignore).
   *
   * @param action input action function
   */
  void setInputAction(InputAction* action);

  /**
   * Critical inputs must be checked for every safety level. Sets an input action list.
   * Such an action is a function (check, ignore).
   *
   * @param actionList list of input action functions
   */
  void setInputActions(std::vector<InputAction*> actionList);

  /**
   * Critical outputs are set automatically for every safety level. Adds an output action
   * to the list of actions. Such an action is a function (set, toggle, leave).
   *
   * @param action input action function
   */
  void setOutputAction(OutputAction* action);

  /**
   * Critical outputs are set automatically for every safety level. Sets an output action list.
   * Such an action is a function (set, toggle, leave).
   *
   * @param actionList list of output action functions
   */
  void setOutputActions(std::vector<OutputAction*> actionList);

  /**
   * Every safety level has an associated level function which is executed while being in this
   * level. Sets the level action to this function.
   *
   * @param action function to be executed while being in this level
   */
  void setLevelAction(std::function<void (SafetyContext* context)> action);

  /**
   * Set a function to run when entering the safety level
   * @param action function to run
   */
  void setEntryAction(std::function<void (SafetyContext* context)> action);

  /**
   * Set a function to run when exiting the safety level
   * @param action function to run
   */
  void setExitAction(std::function<void ()> action);

  /**
   * The level id of this level is compared to another level id.
   *
   * @param level safety level
   *
   * @return true: if this id is smaller than the id of the parameter 'level'
   */
  bool operator<(const SafetyLevel& level);

  /**
   * The level id of this level is compared to another level id.
   *
   * @param level safety level
   *
   * @return true: if this id is smaller than or equal to the id of the parameter 'level'
   */
  bool operator<=(const SafetyLevel& level);

  /**
   * The level id of this level is compared to another level id.
   *
   * @param level safety level
   *
   * @return true: if this id is bigger than the id of the parameter 'level'
   */
  bool operator>(const SafetyLevel& level);

  /**
   * The level id of this level is compared to another level id.
   *
   * @param level safety level
   *
   * @return true: if this id is bigger than or equal to the id of the parameter 'level'
   */
  bool operator>=(const SafetyLevel& level);

  /**
   * The level id of this level is compared to another level id.
   *
   * @param level safety level
   *
   * @return true: if this id is equal to the id of the parameter 'level'
   */
  bool operator==(const SafetyLevel& level);

  /**
   * The level id of this level is compared to another level id.
   *
   * @param level safety level
   *
   * @return true: if this id is not equal to the id of the parameter 'level'
   */
  bool operator!=(const SafetyLevel& level);

 private:
  std::function<void (SafetyContext*)> action;
  int32_t id;
  uint32_t nofActivations;
  std::string description;
  std::map<uint32_t, std::pair<SafetyLevel*, EventType>> transitions;
  std::vector<InputAction*> inputAction;
  std::vector<OutputAction*> outputAction;
  std::function<void (SafetyContext*)> onEntry;
  std::function<void ()> onExit;
  logger::Logger log;
};

/********** Print functions **********/
std::ostream& operator<<(std::ostream& os, eeros::safety::SafetyEvent& event);
std::ostream& operator<<(std::ostream& os, eeros::safety::SafetyEvent* event);
std::ostream& operator<<(std::ostream& os, eeros::safety::SafetyLevel& level);
std::ostream& operator<<(std::ostream& os, eeros::safety::SafetyLevel* level);

}
}
#endif // ORG_EEROS_SAFETY_SAFETYLEVEL_HPP_
