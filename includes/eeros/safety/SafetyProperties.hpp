#ifndef ORG_EEROS_SAFETY_SAFETYPROPORTIES_HPP_
#define ORG_EEROS_SAFETY_SAFETYPROPORTIES_HPP_

#include <stdint.h>

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <vector>

namespace eeros {
namespace safety {

// Forward declarations
class SafetySystem;

/** \brief Safety system.
 *
 * The safety system is configured with safety properties. These properties
 * define safety levels and events for switching between these levels. For your
 * own robot, you have to implement your own set of levels and events by
 * overriding this class.
 */
class SafetyProperties {
  friend class SafetySystem;

 public:
  /**
   * Creates safety properties with no levels initially
   */
  SafetyProperties();

  /**
   * Adds an event to a level. Whenever the safety system is in this level and
   * the specified event is triggered the safety system will switch to the new
   * level. The type of the event might be private or public. Private means the
   * event can only be triggered from within the safety system itself whereas a
   * public event could be triggered from everywhere, notably the control system
   * or a sequence.
   * @param level The safety level for which this event is registered.
   * @param event The safety event to be fired.
   * @param nextLevel The safety level where the event leads to.
   * @param type public or private
   */
  void addEventToLevel(SafetyLevel& level, SafetyEvent event,
                       SafetyLevel& nextLevel, EventType type);

  /**
   * Adds an event to a level and all the levels with a higher identifier.
   * Whenever the safety system is in on of these levels and the specified event
   * is triggered the safety system will switch to the new level. The type of
   * the event might be private or public. Private means the event can only be
   * triggered from within the safety system itself whereas a public event could
   * be triggered from everywhere, notably the control system or a sequence.
   * @param level The lowest safety level for which this event is registered.
   * @param event The safety event to be fired.
   * @param nextLevel The safety level where the event leads to.
   * @param type public or private
   */
  void addEventToLevelAndAbove(SafetyLevel& level, SafetyEvent event,
                               SafetyLevel& nextLevel, EventType type);

  /**
   * Adds an event to a level and all the levels with a lower identifier.
   * Whenever the safety system is in on of these levels and the specified event
   * is triggered the safety system will switch to the new level. The type of
   * the event might be private or public. Private means the event can only be
   * triggered from within the safety system itself whereas a public event could
   * be triggered from everywhere, notably the control system or a sequence.
   * @param level The highest safety level for which this event is registered.
   * @param event The safety event to be fired.
   * @param nextLevel The safety level where the event leads to.
   * @param type public or private
   */
  void addEventToLevelAndBelow(SafetyLevel& level, SafetyEvent event,
                               SafetyLevel& nextLevel, EventType type);

  /**
   * Adds an event to a lower level and an upper level and all the levels in
   * between. Whenever the safety system is in on of these levels and the
   * specified event is triggered the safety system will switch to the new
   * level. The type of the event might be private or public. Private means the
   * event can only be triggered from within the safety system itself whereas a
   * public event could be triggered from everywhere, notably the control system
   * or a sequence.
   * @param lowerLevel The lowest safety level for which this event is
   * registered.
   * @param upperLevel The highest safety level for which this event is
   * registered.
   * @param event The safety event to be fired.
   * @param nextLevel The safety level where the event leads to.
   * @param type public or private
   */
  void addEventToAllLevelsBetween(SafetyLevel& lowerLevel,
                                  SafetyLevel& upperLevel, SafetyEvent event,
                                  SafetyLevel& nextLevel, EventType type);

  /**
   * Getter function for the entry safety level. This is the level from where
   * the safety system starts at the beginning.
   * @return The entry safety level.
   */
  SafetyLevel* getEntryLevel();

  /**
   * Runs a check if critical inputs and outputs are specified for all levels
   * and if the entry level is set.
   * @return true, if everything is ok.
   */
  bool verify();

  /**
   * Adds a new level to the safety system.
   * @param level the new level to add
   */
  void addLevel(SafetyLevel& level);

 protected:
  void setEntryLevel(SafetyLevel& entryLevel);

  std::function<void(SafetyContext*)> exitFunction;
  std::function<void()> abortFunction;
  std::vector<SafetyLevel*> levels;
  std::vector<eeros::hal::OutputInterface*> criticalOutputs;
  std::vector<eeros::hal::InputInterface*> criticalInputs;

 private:
  SafetyLevel* entryLevel;
  uint32_t count;
};

};  // namespace safety
};  // namespace eeros

#endif  // ORG_EEROS_SAFETY_SAFETYPROPORTIES_HPP_
