#ifndef ORG_EEROS_CONTROL_WRAPAROUND_HPP_
#define ORG_EEROS_CONTROL_WRAPAROUND_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <mutex>
#include <concepts>
#include <type_traits>
#include <cmath>


namespace eeros {
namespace control {

/**
 * A WrapAround block wraps an input value between two limit values.
 * As soon as the input value exceeds an upper limit, the output will wrap
 * around and will be set to minVal. The wrap direction works in positive or 
 * direction. The output value will always vary between minVal and maxVal.
 * 
 * @tparam T - signal input and output type (double - default type)
 * @tparam Twrap - type of min and max values for wrap. Must be double, or same as Tout (double - default type) 
 * @tparam U - signal unit type (dimensionless - default type)
 *  
 * @since 1.3
 */

template<typename T = double, typename Twrap = T, SIUnit U = SIUnit::create()>
class WrapAround : public Blockio<1,1,T,T,MakeUnitArray<U>::value,MakeUnitArray<U>::value> {
 public:   
  /**
   * Constructs a WrapAround instance specifying minValue and maxValue of output 
   * to realize wrap. \n
   *
   * @param min - minimum value for wrap around
   * @param max - maximum value for wrap around
   */
  explicit WrapAround(Twrap min, Twrap max) : enabled(true), minVal(min), maxVal(max) { }
      
  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  WrapAround(const WrapAround& s) = delete;
  WrapAround& operator=(const WrapAround&) = delete;
  
  /**
   * Runs the wrap around algorithm, as described above.
   */
  void run() override {
    std::lock_guard<std::mutex> lock(mtx);
    T inVal = this->in.getSignal().getValue();
    T outVal = enabled ? calculateResult(inVal) : inVal;
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
    this->minVal = min;
    this->maxVal = max;
  }

 private:
  // Arithmetic types (double, int, float, etc.)
  T calculateResult(T inValue) requires std::is_arithmetic_v<T> && std::is_arithmetic_v<Twrap> {
    double delta = std::fabs(maxVal - minVal);
    double num = inValue - minVal;
    double tquot = std::floor(num / delta);
    T outVal = num - tquot * delta;
    if (outVal < 0) outVal = outVal + delta;
    else if (outVal > 0) outVal = outVal + minVal;
    else outVal = inValue;
    return outVal;
  }

  // Compound types (vectors, matrices) with scalar wrap limits
  T calculateResult(T inValue) requires (!std::is_arithmetic_v<T>) && std::is_arithmetic_v<Twrap> {
    T outVal;
    for (unsigned int i = 0; i < inValue.size(); i++) {
      double delta = std::fabs(maxVal - minVal);
      double num = inValue[i] - minVal;
      double tquot = std::floor(num / delta);
      outVal[i] = num - tquot * delta;
      if (outVal[i] < 0) outVal[i] = outVal[i] + delta;
      else if (outVal[i] > 0) outVal[i] = outVal[i] + minVal;
      else outVal[i] = inValue[i];
    }
    return outVal;
  }

  // Compound types (vectors, matrices) with vector wrap limits
  T calculateResult(T inValue) requires (!std::is_arithmetic_v<T>) && (!std::is_arithmetic_v<Twrap>) {
    T outVal;
    for (unsigned int i = 0; i < inValue.size(); i++) {
      double delta = std::fabs(maxVal[i] - minVal[i]);
      double num = inValue[i] - minVal[i];
      double tquot = std::floor(num / delta);
      outVal[i] = num - tquot * delta;
      if (outVal[i] < 0) outVal[i] = outVal[i] + delta;
      else if (outVal[i] > 0) outVal[i] = outVal[i] + minVal[i];
      else outVal[i] = inValue[i];
    }
    return outVal;
  }

  /*
   * Friend operator overload to give the operator overload outside
   * the class access to the private fields.
   */
  template<typename Xout, typename Xwrap>
  friend std::ostream &operator<<(std::ostream &os, const WrapAround<Xout, Xwrap> &wrap);

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
std::ostream &operator<<(std::ostream &os, const WrapAround<Tout, Twrap> &wrap) {
  os << "Block WrapAround: '" << wrap.getName() << "' is enabled=" << wrap.enabled;
  os << ", minVal=" << wrap.minVal << ", maxVal=" << wrap.maxVal;
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_WRAPAROUND_HPP_ */
