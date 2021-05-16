#ifndef ORG_EEROS_CONTROL_SWITCH_HPP_
#define ORG_EEROS_CONTROL_SWITCH_HPP_

#include <eeros/control/Block1o.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>
#include <mutex>

namespace eeros {
namespace control {

/**
 * A switch allows to select a signal from severel inputs to be routed to the output.
 * If a signal at a chosen input is close to a predefined value, the switch can automatically switch 
 * to a predefined position and a safety event can be triggered. The switch can be made to switch
 * only if the current safety level is greater or the same as the active level set on this switch.
 * Two or more switches can be combined. This mechanism allows to switch them simultaneously.
 * 
 * @tparam N - number of inputs
 * @tparam T - value type (double - default type)
 * 
 * @since v0.4
 */

template < uint8_t N = 2, typename T = double >
class Switch : public Block1o<T> {
 public:

  /**
   * Constructs a switch instance with the switch initialized to position 0.\n
   */
  Switch() : Switch(0) { }

  /**
   * Constructs a Switch instance with the switch initialized to a given position.\n
   * 
   * @param initInputIndex - switch is initially set to this position
   */
  Switch(uint8_t initInputIndex) 
      : currentInput(initInputIndex),
        safetySystem(nullptr),
        safetyEvent(nullptr),
        activeLevel(nullptr),
        log(logger::Logger::getLogger()) {
    for(uint8_t i = 0; i < N; i++) in[i].setOwner(this);
  }

  /**
  * Disabling use of copy constructor because the block should never be copied unintentionally.
  */
  Switch(const Switch& s) = delete; 

  /**
  * Runs the switch block.
  */
  virtual void run() override {
    std::lock_guard<std::mutex> lock(mtx);
    auto val = this->in[currentInput].getSignal().getValue();
    if (armed && !switched) {
      if (val < (switchLevel + delta) && val > (switchLevel - delta)) {
        if (activeLevel == nullptr ||
           (activeLevel != nullptr && safetySystem->getCurrentLevel() >= *activeLevel)
           ) {
          log.warn() << "Switch \'" + this->getName() + "\' switches!";
          switchToInput(nextInput);
          for(Switch* i : c) i->switchToInput(nextInput);
          switched = true;
          armed = false;
          if (safetySystem != nullptr && safetyEvent != nullptr) {
            safetySystem->triggerEvent(*safetyEvent);
          }
        }
      }
    }
    this->out.getSignal().setValue(this->in[currentInput].getSignal().getValue());
    this->out.getSignal().setTimestamp(this->in[currentInput].getSignal().getTimestamp());
  }
                  
  /**
  * Getter function for the input with a given index.
  * 
  * @param index - index of input
  * @return The input with this index
  */
  virtual Input<T>& getIn(uint8_t index) {
    return in[index];
  }
                                          
  /**
  * Changes the switch position.
  * 
  * @param index - position to switch to
  * @return true, if operation successful
  */
  virtual bool switchToInput(uint8_t index) {
    if(index >= 0 && index < N) {
      currentInput = index;
      for(Switch* i : c) i->switchToInput(index);
      return true;
    }
    return false;
  }
                  
  /**
  * Gets the actual position of the switch.
  * 
  * @return current position
  */
  virtual uint8_t getCurrentInput() const {
    return currentInput;
  }
                  
  /**
  * Registers a safety system together with a safety event, which will be
  * triggered as soon as auto switching takes place and the trigger was armed beforehand.
  * @see setCondition()
  * 
  * @param ss - safety system
  * @param e - safety event
  */
  virtual void registerSafetyEvent(safety::SafetySystem& ss, safety::SafetyEvent& e) {
    safetySystem = &ss;
    safetyEvent = &e;
  }

  /**
   * Sets the active safety level on this switch. The switch will automatically
   * switch if a switching condition is set and the current safety level is equal 
   * or greater than the level set with this function.
   * @see setCondition()
   *
   * @param ss - safety system
   * @param level - SafetyLevel
   */
  virtual void setActiveLevel(safety::SafetySystem& ss, safety::SafetyLevel &level) {
    std::lock_guard<std::mutex> lock(mtx);
    safetySystem = &ss;
    activeLevel = &level;
  }

  /**
  * Configure the switch so that it switches only after the input signal is close 
  * to a given level within a certain margin, that is when the following condition is met:
  * switchLevel - delta < signal < switchLevel + delta
  * Switching will then happen automatically if the switch is armed.
  * It is possible to trigger a safety event upon switching
  * @see registerSafetyEvent()
  * 
  * @param switchLevel - level, that the input signal has to reach
  * @param delta - level margin, that the input signal has to reach
  * @param index - position to switch to 
  */
  virtual void setCondition(T switchLevel, T delta, uint8_t index) {
    this->switchLevel = switchLevel;
    this->delta = delta;
    nextInput = index;
  }
                  
  /**
  * Allows to switch automatically.
  * @see setCondition()
  */
  virtual void arm() {
    armed = true;
    switched = false;
  }

  /**
  * Reads the state of the trigger mechanism.
  * @see setCondition()
  * 
  * @return true, if safety event was fired
  */
  virtual bool triggered() const {
    return switched;
  }
                  
  /**
  * Connects a second switch to this switch. The second switch will 
  * automatically be on the same position as the this switch.
  * triggered as soon as auto switching takes place.
  * 
  * @param s - switch to be connected to this switch
  */
  virtual void combine(Switch& s) {
    c.push_back(&s);
    s.switchToInput(currentInput);
  }

 protected:
  Input<T> in[N];
  uint8_t currentInput, nextInput;
  T switchLevel, delta;
  bool armed = false;
  bool switched = false;
  safety::SafetySystem* safetySystem;
  safety::SafetyEvent* safetyEvent;
  safety::SafetyLevel *activeLevel;
  std::vector<Switch*> c;
  eeros::logger::Logger log;
  std::mutex mtx{};
};

/********** Print functions **********/
template <uint8_t N, typename T>
std::ostream& operator<<(std::ostream& os, Switch<N,T>& sw) {
  os << "Block switch: '" << sw.getName() << "'"; 
  return os;
}

};
};

#endif /* ORG_EEROS_CONTROL_SWITCH_HPP_ */
