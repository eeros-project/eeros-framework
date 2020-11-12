#ifndef ORG_EEROS_CONTROL_I_HPP_
#define ORG_EEROS_CONTROL_I_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <eeros/logger/Logger.hpp>
#include <cmath>

namespace eeros {
namespace control {

/**
 * An integrator block is used to integrate an input signal. 
 * The block can be enabled or disabled. When disabled the block no longer 
 * integrates and keeps its actual state. An integrator must have its
 * initial state set to a defined level. Further, the block allows to
 * set an upper and lower limit to allow for anti windup.
 *
 * @tparam T - output type (double - default type)
 * @since v0.6
 */
template < typename T = double >
class I: public Block1i1o<T> {
 public:
  /**
   * Constructs an integrator instance.\n
   */
  I() : first(true), enabled(false) {prev.clear(); clearLimits(); }
 
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  I(const I& s) = delete; 

  /**
   * Runs the integration algorithm if the block is enabled, else
   * the current state is left unchanged.
   *
   * @see setInitCondition(T val)
   * @see setLimit(T upper, T lower)
   * @see enable()
   * @see disable()
   */
  virtual void run() {
    double tin = this->in.getSignal().getTimestamp() / 1000000000.0;
    double tprev = this->prev.getTimestamp() / 1000000000.0;
    
    double dt;
    if (first) {
      dt = 0; 
      first = false;
    } else dt = (tin - tprev);
    T valin = this->in.getSignal().getValue();
    T valprev = this->prev.getValue();
    T output;
    if (enabled) {
      T val = valprev + valin * dt;
      if ((val < upperLimit) && (val > lowerLimit)) output = val; 
      else output = valprev;
    } else output = valprev;
    this->out.getSignal().setValue(output);
    this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
    this->prev = this->out.getSignal();
  }

  /**
   * Enables the integrator.
   *
   * If enabled, run() will integrate.
   *
   * @see disable()
   */
  virtual void enable() {
    this->enabled = true;
  }

  /**
   * Disables the integrator.
   *
   * If disabled, run() will not integrate and keep the current state.
   *
   * @see enable()
   */
  virtual void disable() {
    this->enabled = false;
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
   * The integrator will not integrate above the upper or below
   * the lower limit but will stay at this limiting values.
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

  /*
   * Friend operator overload to give the operator overload outside
   * the class access to the private fields.
   */
  template<typename X>
  friend std::ostream &operator<<(std::ostream &os, I<X> &i);

 protected:
//  eeros::logger::Logger log;
  bool first;
  bool enabled;
  Signal<T> prev;
  T upperLimit, lowerLimit;
 private:
  virtual void clearLimits() {
    _clear<T>();
  }
  template <typename S> typename std::enable_if<std::is_integral<S>::value>::type _clear() {
    upperLimit = std::numeric_limits<int32_t>::max();
    lowerLimit = -upperLimit;
  }
  template <typename S> typename std::enable_if<std::is_floating_point<S>::value>::type _clear() {
    upperLimit = std::numeric_limits<double>::max();
    lowerLimit = -upperLimit;
  }
  template <typename S> typename std::enable_if<!std::is_arithmetic<S>::value && std::is_integral<typename S::value_type>::value>::type _clear() {
    upperLimit.fill(std::numeric_limits<int32_t>::max());
    lowerLimit = -upperLimit;
  }
  template <typename S> typename std::enable_if<   !std::is_arithmetic<S>::value && std::is_floating_point<typename S::value_type>::value>::type _clear() {
    upperLimit.fill(std::numeric_limits<double>::max());
    lowerLimit = -upperLimit;
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

};
};
#endif /* ORG_EEROS_CONTROL_I_HPP_ */
