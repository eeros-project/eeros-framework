#ifndef ORG_EEROS_CONTROL_WRAPAROUND_HPP_
#define ORG_EEROS_CONTROL_WRAPAROUND_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <mutex>

using namespace eeros::math;

namespace eeros {
namespace control {
  
/**
 * A WrapAround block wraps an input value between two limit values.
 * As soon as the input value exceeds an upper limit, the output will wrap
 * around and will be set to minVal. The wrap direction works in positive or 
 * direction. The output value will always vary between minVal and maxVal.
 * 
 * @tparam Tout - output type (double - default type) 
 * @tparam Twrap - type of min and max values for wrap. Must be double, or same as Tout (double - default type) 
 * @tparam minVal: minimum value - lower value of output signal
 * @tparam maxVal: maximum value - higher value of output signal
 *  
 * @since 1.3
 */

template<typename Tout = double, typename Twrap = Tout>
class WrapAround : public Blockio<1,1,Tout> {
 public:   
  /**
   * Constructs a WrapAround instance specifying minValue and maxValue of output 
   * to realize wrap. \n
   *
   * @param min - minimum value for wrap around
   * @param max - maximum value for wrap around
   */
  WrapAround(Twrap min, Twrap max) : enabled(true), minVal(min), maxVal(max) { }
      
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  WrapAround(const WrapAround& s) = delete; 
  
  /**
   * Runs the wrap around algorithm, as described above.
   */
  virtual void run() override {
    std::lock_guard<std::mutex> lock(mtx);
    Tout inVal = this->in.getSignal().getValue();
    Tout outVal = inVal;
    if (enabled) outVal = calculateResult<Tout>(inVal);
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
  virtual void enable() override {
    enabled = true;
  }

  /**
   * Disables the block.
   * 
   * If disabled, run() will set output = input.
   * 
   * @see enable()
   */
  virtual void disable() override {
    enabled = false;
  }

  /**
   * Sets min and max values.
   * 
   * @param min - minimum value for wrap around
   * @param max - maximum value for wrap around
   */
  virtual void setMinMax(Twrap min, Twrap max) {
    std::lock_guard<std::mutex> lock(mtx);
    this->minVal = minVal;
    this->maxVal = maxVal;
  }

 private:
  template <typename S> 
  typename std::enable_if<std::is_arithmetic<S>::value, S>::type calculateResult(S inValue) {
    Tout outVal;
    double delta = fabs(maxVal - minVal);
    double num = inValue - this->minVal;
    double den = delta;
    double tquot = floor(num / den);
    outVal = num - tquot * den;
    if (outVal < 0) outVal = outVal + delta;
    else if (outVal > 0) outVal = outVal + this->minVal; 
    else outVal = inValue;
    return outVal;
  }

  template <typename S> 
  typename std::enable_if<std::is_compound<S>::value && std::is_arithmetic<Twrap>::value, S>::type calculateResult(S inValue) {
    Tout outVal;
    for(unsigned int i = 0; i < inValue.size(); i++) {
      double delta = fabs(maxVal - minVal);
      double num = inValue[i] - this->minVal;
      double den = delta;
      double tquot = floor(num / den);
      outVal[i] = num - tquot * den;
      if (outVal[i] < 0) outVal[i] = outVal[i] + delta;
      else if ((outVal[i] > 0)) outVal[i] = outVal[i] + this->minVal; 
      else outVal[i] = inValue[i];
    }
    return outVal;
  }
  
  template <typename S> 
  typename std::enable_if<std::is_compound<S>::value && std::is_compound<Twrap>::value, S>::type calculateResult(S inValue) {
    Tout outVal;
    for(unsigned int i = 0; i < inValue.size(); i++) {
      double delta = fabs(maxVal[i] - minVal[i]);
      double num = inValue[i] - this->minVal[i];
      double den = delta;
      double tquot = floor(num/den);
      outVal[i] = num - tquot * den;
      if (outVal[i] < 0) outVal[i] = outVal[i] + delta;
      else if ((outVal[i] > 0)) outVal[i] = outVal[i] + this->minVal[i]; 
      else outVal[i] = inValue[i];
    }
    return outVal;
  }

  /*
   * Friend operator overload to give the operator overload outside
   * the class access to the private fields.
   */
  template<typename Xout, typename Xwrap>
  friend std::ostream &operator<<(std::ostream &os, WrapAround<Xout, Xwrap> &wrap);

 protected:
  bool enabled{true};
  std::mutex mtx;
  Twrap minVal;
  Twrap maxVal;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * WarpAround instance to an output stream.\n
 * Does not print a newline control character.
 */
template<typename Tout, typename Twrap>
std::ostream &operator<<(std::ostream &os, WrapAround<Tout, Twrap> &wrap) {
  os << "Block WrapAround: '" << wrap.getName() << "' is enabled=" << wrap.enabled;
  os << ", minVal=" << wrap.minVal << ", maxVal=" << wrap.maxVal;
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_WRAPAROUND_HPP_ */
