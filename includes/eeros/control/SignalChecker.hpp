#ifndef ORG_EEROS_CONTROL_SIGNALCHECKER_HPP_
#define ORG_EEROS_CONTROL_SIGNALCHECKER_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>
#include <type_traits>
#include <memory>
#include <mutex>


namespace eeros {
namespace control {

/**
 * A signal checker block is used to check if a signal is between a lower
 * and upper limit. It can also check whether a signal is out off this range.
 * If it exceeds the limits, a safety event is triggered.
 * The safety event is triggered only if a safety event is registered and the
 * current safety level is greater or the same as the active level set on this
 * signal checker.
 * If a safety event was already triggered, the signal checker must be reset
 * before it will trigger another safety event.
 * @see reset()
 *
 *
 * SignalChecker is a class template with two type and one non-type
 * template arguments.
 * The two type template arguments specify the types which are used for the
 * signal type and the limit type when the class template is instantiated.
 * The non-type template argument enables the usage of the norm() method
 * on the signal type. If set to true, the return value of the norm() method
 * is limit checked. This is for example useful with the class Matrix when
 * the norm of a vector must be limit checked.
 *
 * A signal checker block is suitable for use with multiple threads.
 *
 * @tparam Tsig - signal type (double - default type)
 * @tparam Tlim - limit type (Tsig - default type)
 * @tparam checkNorm - use of the norm() method (false - default value)
 *
 * @since v0.6
 */

template<typename Tsig = double, typename Tlim = Tsig, bool checkNorm = false>
class SignalChecker : public Blockio<1,0,Tsig> {

 public:
  /**
   * Constructs a signal checker instance with a lower and upper limit.\n
   *
   * @param lowerLimit - initial lower limit value
   * @param upperLimit - initial upper limit value
   * @param offRange - checks that the signal is lower than the lower limit or greater than the upper limit
   */
  SignalChecker(Tlim lowerLimit, Tlim upperLimit, bool offRange = false) 
      : lowerLimit(lowerLimit),
        upperLimit(upperLimit),
        fired(false),
        safetySystem(nullptr),
        safetyEvent(nullptr),
        activeLevel(nullptr),
        log(logger::Logger::getLogger()), 
        offRange(offRange) {}

  /**
   * Runs the checker algorithm.
   *
   * Checks if the signal is in the band in between lower and upper limit.
   *
   * Triggers a safety event otherwise.
   *
   * The safety event is triggered only if:\n
   *  * a safety event is registered\n
   *  * the current safety level is greater or the same
   *    as the active level set on this signal checker
   *
   *
   * @see registerSafetyEvent()
   * @see setActiveLevel()
   */
  virtual void run() override {
    std::lock_guard<std::mutex> lock(mtx);

    auto val = this->in.getSignal().getValue();
    if (!fired) {
      if (offRange) {
        if (withinLimits<bool>(val)) {
          if (safetySystem != nullptr && safetyEvent != nullptr) {
            if (activeLevel == nullptr ||
            (activeLevel != nullptr && safetySystem->getCurrentLevel() >= *activeLevel)
            ) {
              log.warn() << "Signal checker \'" + this->getName() + "\' fires!";
              safetySystem->triggerEvent(*safetyEvent);
              fired = true;
            }
          }
        }
      } else {
        if (limitsExceeded<bool>(val)) {
          if (safetySystem != nullptr && safetyEvent != nullptr) {
            if (activeLevel == nullptr ||
            (activeLevel != nullptr && safetySystem->getCurrentLevel() >= *activeLevel)
            ) {
              log.warn() << "Signal checker \'" + this->getName() + "\' fires!";
              safetySystem->triggerEvent(*safetyEvent);
              fired = true;
            }
          }
        }
      }
    }
  }


  /**
   * Sets the lower and upper limit.
   *
   * @param lowerLimit - lower limit value
   * @param upperLimit - upper limit value
   */
  virtual void setLimits(Tlim lowerLimit, Tlim upperLimit) {
    std::lock_guard<std::mutex> lock(mtx);
    this->lowerLimit = lowerLimit;
    this->upperLimit = upperLimit;
  }


  /**
   * Resets the checker so it can fire a safety event again.
   */
  virtual void reset() {
    std::lock_guard<std::mutex> lock(mtx);
    fired = false;
  }


  /**
   * Registers a safety event.
   *
   * @param ss - SafetySystem
   * @param e - SafetyEvent
   */
  virtual void registerSafetyEvent(safety::SafetySystem &ss, safety::SafetyEvent &e) {
    std::lock_guard<std::mutex> lock(mtx);
    safetySystem = &ss;
    safetyEvent = &e;
  }


  /**
   * Sets the active safety level on this signal checker.
   *
   * @param level - SafetyLevel
   */
  virtual void setActiveLevel(safety::SafetyLevel &level) {
    std::lock_guard<std::mutex> lock(mtx);
    activeLevel = &level;
  }


 protected:
  Tlim lowerLimit, upperLimit;
  bool fired;
  safety::SafetySystem *safetySystem;
  safety::SafetyEvent *safetyEvent;
  safety::SafetyLevel *activeLevel;
  eeros::logger::Logger log;
  std::mutex mtx{};
  bool offRange;

 private:
  template<typename S>
  typename std::enable_if<!checkNorm, S>::type limitsExceeded(Tsig value) {
    return !(value > lowerLimit && value < upperLimit);
  }

  template<typename S>
  typename std::enable_if<checkNorm, S>::type limitsExceeded(Tsig value) {
    return !(value.norm() > lowerLimit && value.norm() < upperLimit);
  }

  template<typename S>
  typename std::enable_if<!checkNorm, S>::type withinLimits(Tsig value) {
    return (value > lowerLimit && value < upperLimit);
  }

  template<typename S>
  typename std::enable_if<checkNorm, S>::type withinLimits(Tsig value) {
    return (value.norm() > lowerLimit && value.norm() < upperLimit);
  }

};

}
}

#endif /* ORG_EEROS_CONTROL_SIGNALCHECKER_HPP_ */
