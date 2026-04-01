#ifndef ORG_EEROS_CONTROL_SATURATION_HPP_
#define ORG_EEROS_CONTROL_SATURATION_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <mutex>
#include <ostream>

namespace eeros::control {

/**
 * Limits an input signal between a lower and upper bound.
 *
 * When enabled, the output is clamped to [lowerLimit, upperLimit].
 * When disabled, the output follows the input unchanged.
 *
 * For scalar types the limits are compared directly. For vector types
 * the clamping is applied element-wise.
 *
 * @tparam T - input/output type (default: double)
 *
 * @since v1.2
 */
template < typename T = double >
class Saturation : public Blockio<1,1,T> {
 public:
  /**
   * Constructs a Saturation with separate lower and upper limits.
   *
   * @param lower - lower limit
   * @param upper - upper limit
   */
  Saturation(T lower, T upper) : lowerLimit(lower), upperLimit(upper) {}
  
  /**
   * Constructs a Saturation with a symmetric limit [-lim, lim].
   *
   * @param lim - symmetric limit
   */
  explicit Saturation(T lim) : Saturation(-lim, lim) {}

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  Saturation(const Saturation& s) = delete; 
  Saturation& operator=(const Saturation&) = delete;

  /**
   * Runs the saturation algorithm.
   *
   * Clamps the input to [lowerLimit, upperLimit] when enabled,
   * otherwise passes the input through unchanged.
   */
  virtual void run() {
    std::lock_guard<std::mutex> lock(mtx);
    T inVal = this->getIn().getSignal().getValue();
    T outVal = enabled ? calculate(inVal) : inVal;
    this->getOut().getSignal().setValue(outVal);
    this->getOut().getSignal().setTimestamp(this->getIn().getSignal().getTimestamp());
  }
  
  /**
   * Enables the block.
   * 
   * If enabled, run() will perform wrap around.
   * 
   * @see disable()
   */
  virtual void enable() {
    enabled = true;
  }
  
  /**
   * Disables the block.
   * 
   * If disabled, run() will set output = input.
   * 
   * @see enable()
   */
  virtual void disable() {
    enabled = false;
  }
  
  /**
   * Sets lower and upper limit values.
   * 
   * @param lower - lower limit
   * @param upper - upper limit
   */
  virtual void setLimit(T lower, T upper) {
    lowerLimit = lower;
    upperLimit = upper;
  }
  
 private:
  T lowerLimit, upperLimit;
  bool enabled{true};
  std::mutex mtx;

  // scalar
  T calculate(T inVal) requires std::is_arithmetic_v<T> {
    if (inVal > upperLimit) return upperLimit;
    if (inVal < lowerLimit) return lowerLimit;
    return inVal;
  }

  // vector — element-wise
  T calculate(T inVal) requires (!std::is_arithmetic_v<T>) {
    T outVal = inVal;
    for (unsigned int i = 0; i < inVal.size(); ++i) {
      if (inVal[i] > upperLimit[i]) outVal[i] = upperLimit[i];
      if (inVal[i] < lowerLimit[i]) outVal[i] = lowerLimit[i];
    }
    return outVal;
  }
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * saturation instance to an output stream.\n
 * Does not print a newline control character.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const Saturation<T>& s) {
  os << "Block saturation: '" << s.getName() << "' lower limit=" << s.lowerLimit << ", upper limit=" << s.upperLimit; 
  return os;
}

}

#endif /* ORG_EEROS_CONTROL_SATURATION_HPP_ */
