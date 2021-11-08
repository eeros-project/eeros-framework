#ifndef ORG_EEROS_CONTROL_SATURATION_HPP_
#define ORG_EEROS_CONTROL_SATURATION_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <mutex>
#include <type_traits>

namespace eeros {
namespace control {

/**
 * A Saturation block limits an input value between two limit values.
 * The output value will always vary between lower and upper limit.
 * If the block is disabled, the output value will simply follow the input.
 * 
 * @tparam T - output type (double - default type) 
 *  
 * @since 1.2
 */

template < typename T = double >
class Saturation : public Blockio<1,1,T> {
 public:
  /**
   * Constructs a Saturation instance specifying lower and upper limit.\n
   *
   * @param lower - lower limit
   * @param upper - upper limit
   */
  Saturation(T lower, T upper) : enabled(true) {
    lowerLimit = lower;
    upperLimit = upper;
  }
  
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Saturation(const Saturation& s) = delete; 

  /**
   * Runs the saturation algorithm, as described above.
   */
  virtual void run() {
    std::lock_guard<std::mutex> lock(mtx);
    T inVal = this->in.getSignal().getValue();
    T outVal = inVal;
    if (enabled) outVal = calculateResult<T>(inVal);
    this->out.getSignal().setValue(outVal);
    this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
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
  template <typename S> 
  typename std::enable_if<std::is_arithmetic<S>::value, S>::type calculateResult(S inVal) {
    T outVal = inVal;
    if (inVal > upperLimit) outVal = upperLimit;
    if (inVal < lowerLimit) outVal = lowerLimit;
    return outVal;
  }

  template <typename S> 
  typename std::enable_if<std::is_compound<S>::value, S>::type calculateResult(S inVal) {
    T outVal = inVal;
    for (unsigned int i = 0; i < outVal.size(); i++) {
      if (inVal[i] > upperLimit[i]) outVal[i] = upperLimit[i];
      if (inVal[i] < lowerLimit[i]) outVal[i] = lowerLimit[i];
    }   
    return outVal;
  }

  T lowerLimit, upperLimit;
  bool enabled;
  std::mutex mtx;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * saturation instance to an output stream.\n
 * Does not print a newline control character.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, Saturation<T>& s) {
  os << "Block saturation: '" << s.getName() << "' lower limit=" << s.lowerLimit << ", upper limit=" << s.upperLimit; 
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_SATURATION_HPP_ */
