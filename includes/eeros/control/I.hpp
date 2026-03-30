#ifndef ORG_EEROS_CONTROL_I_HPP_
#define ORG_EEROS_CONTROL_I_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/safety/SafetyLevel.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <cmath>
#include <limits>
#include <mutex>
#include <ostream>

namespace eeros::control {

/**
 * @brief Integrator block for integrating an input signal.
 *
 * The block can be enabled or disabled. When disabled it stops integrating
 * and holds its current state. An upper and lower limit can be configured
 * for anti-windup. Enabling can be tied to a @ref safety::SafetyLevel —
 * the integrator runs only when the current level is equal to or greater
 * than the configured active level.
 *
 * @tparam T  Signal type (default: @c double). Supports arithmetic types
 *            and composite types with a @c value_type member (e.g. matrices).
 *
 * @since v0.6
 */
template < typename T = double >
class I: public Blockio<1,1,T> {
 public:
  /**
   * @brief Constructs an integrator with limits set to the full numeric range.
   */
  I() {
    prev.clear(); 
    clearLimits();
  }
 
  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  I(const I& s) = delete; 
  I& operator=(const I&) = delete;

  /**
   * @brief Integrates the input signal if enabled, otherwise holds current state.
   *
   * @see setInitCondition()
   * @see setLimit()
   * @see enable()
   * @see disable()
   */
  void run() override {
    std::lock_guard lock(mtx);
    if (activeLevel != nullptr)
      enabled =  safetySystem->getCurrentLevel() >= *activeLevel;
    const double tin = this->getIn().getSignal().getTimestamp() / 1e9;
    const double tprev = this->prev.getTimestamp() / 1e9;
    const double dt = first ? (first = false, 0.0) : (tin - tprev);
    const T valin = this->getIn().getSignal().getValue();
    const T valprev = this->prev.getValue();
    T output = valprev;
    if (enabled) {
      T val = valprev + valin * dt;
      if (val < upperLimit && val > lowerLimit) output = val;
    }
    this->getOut().getSignal().setValue(output);
    this->getOut().getSignal().setTimestamp(this->getIn().getSignal().getTimestamp());
    this->prev = this->getOut().getSignal();
  }

  /**
   * Enables the integrator.
   * If enabled, run() will integrate.
   *
   * @see disable()
   */
  virtual void enable() {
    enabled = true;
  }

  /**
   * Disables the integrator.
   *
   * If disabled, run() will not integrate and keep the current state.
   *
   * @see enable()
   */
  virtual void disable() {
    enabled = false;
  }
  
  /**
   * Set the initial state of the integrator from where the integrator
   * will start integrating when enabled.
   *
   * @see enable()
   * @param val - initial state
   */
  virtual void setInitCondition(T val) {
    this->prev.setValue(val);
  }
  
  /**
   * Set the upper and lower limit of the integrator.
   *
   * The integrator clamps its output to [@p lower, @p upper].
   * If the current state already exceeds the new limits it is clamped immediately.
   *
   * @param upper - upper limit
   * @param lower - lower limit
   */
  virtual void setLimit(T upper, T lower) {
    this->upperLimit = upper;
    this->lowerLimit = lower;
    T val = prev.getValue();
    if (val > upper) prev.setValue(upper);
    if (val < lower) prev.setValue(lower);
  }

  /**
   * Sets the active safety level on this integrator. The integrator will only run 
   * if the current safety level is equal or greater than the level set with this function.
   *
   * @param ss - safety system
   * @param level - SafetyLevel
   */
  virtual void setActiveLevel(safety::SafetySystem& ss, safety::SafetyLevel &level) {
    std::lock_guard lock(mtx);
    safetySystem = &ss;
    activeLevel = &level;
  }

  /*
   * Friend operator overload to give the operator overload outside
   * the class access to the private fields.
   */
  template<typename X>
  friend std::ostream &operator<<(std::ostream &os, I<X> &i);

 protected:
  bool first{true};
  bool enabled{false};
  Signal<T> prev{};
  T upperLimit{}, lowerLimit{};
  safety::SafetySystem *safetySystem{nullptr};
  safety::SafetyLevel *activeLevel{nullptr};
  std::mutex mtx{};
  
 private:
  void clearLimits() {
    if constexpr (std::is_arithmetic_v<T>) {
      upperLimit = std::numeric_limits<T>::max();
      lowerLimit = std::numeric_limits<T>::lowest();
    } else {
      // composite type (e.g. Matrix) — fill each element
      using V = typename T::value_type;
      upperLimit.fill(std::numeric_limits<V>::max());
      lowerLimit.fill(std::numeric_limits<V>::lowest());
    }
  }

};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * Integrator instance to an output stream.\n
 * Does not print a newline control character.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, I<T>& i) {
  os << "Block integrator: '" << i.getName() << "' is enabled=" << i.enabled;
  os << ", upperLimit=" << i.upperLimit << ", lowerLimit=" << i.lowerLimit; 
  return os;
}

}

#endif /* ORG_EEROS_CONTROL_I_HPP_ */
