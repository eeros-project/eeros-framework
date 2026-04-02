#ifndef ORG_EEROS_CONTROL_WRAPAROUND_HPP_
#define ORG_EEROS_CONTROL_WRAPAROUND_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <cmath>
#include <mutex>
#include <ostream>

namespace eeros::control {

/**
 * @brief Wraps an input signal into a [min, max] interval.
 *
 * When the input exceeds @c maxVal it wraps back to @c minVal, and vice versa.
 * Works in both positive and negative directions. The output always stays
 * within [@c minVal, @c maxVal].
 *
 * @c Twrap may be:
 * - the same as @c Tout (per-element min/max for composite types)
 * - @c double when @c Tout is a composite type (scalar min/max applied to all elements)
 *
 * @tparam Tout   Output signal type (default: @c double)
 * @tparam Twrap  Type of the min/max limits (default: same as @c Tout)
 *
 * @since v1.3
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
  WrapAround(Twrap min, Twrap max) : minVal(min), maxVal(max) { }
      
  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  WrapAround(const WrapAround&)            = delete;
  WrapAround& operator=(const WrapAround&) = delete;
  
  /**
   * @brief Applies the wrap-around algorithm to the input signal.
   */
  void run() override {
    std::lock_guard lock(mtx);
    const Tout inVal = this->getIn().getSignal().getValue();
    this->getOut().getSignal().setValue(enabled ? wrapped(inVal) : inVal);
    this->getOut().getSignal().setTimestamp(this->getIn().getSignal().getTimestamp());
  }

  /**
   * @brief Enables wrap-around processing. @see disable()
   */
  virtual void enable() {
    enabled = true;
  }

  /**
   * @brief Disables wrap-around — output passes through unchanged. @see enable()
   */
  virtual void disable() {
    enabled = false;
  }

  /**
   * @brief Updates the wrap limits.
   * 
   * @param min - minimum value for wrap around
   * @param max - maximum value for wrap around
   */
  virtual void setMinMax(Twrap min, Twrap max) {
    std::lock_guard lock(mtx);
    minVal = min;
    maxVal = max;
  }

  /*
   * Friend operator overload to give the operator overload outside
   * the class access to the private fields.
   */
  template<typename Xout, typename Xwrap>
  friend std::ostream &operator<<(std::ostream &os, const WrapAround<Xout, Xwrap> &wrap);

 protected:
  bool enabled{true};
  std::mutex mtx{};
  Twrap minVal;
  Twrap maxVal;

 private:
  // ── Single wrap computation for one scalar element ───────────────────────
  static double wrapScalar(double val, double lo, double hi) {
    const double delta = std::fabs(hi - lo);
    const double num = val - lo;
    const double tquot = std::floor(num / delta);
    double out = num - tquot * delta;
    if (out < 0.0) out += delta;
    else if (out > 0.0) out += lo;
    else out  = val;
    return out;
  }

  // Arithmetic Tout
  Tout wrapped(Tout inVal) const requires std::is_arithmetic_v<Tout> {
    return static_cast<Tout>(wrapScalar(inVal, minVal, maxVal));
  }

  // Compound Tout, scalar Twrap (same limit for every element)
  Tout wrapped(Tout inVal) const
      requires (!std::is_arithmetic_v<Tout> && std::is_arithmetic_v<Twrap>) {
    Tout out{};
    for (unsigned int i = 0; i < inVal.size(); ++i)
      out[i] = wrapScalar(inVal[i], minVal, maxVal);
    return out;
  }

  // ── Compound Tout, compound Twrap (per-element limits) 
  Tout wrapped(Tout inVal) const
      requires (!std::is_arithmetic_v<Tout> && !std::is_arithmetic_v<Twrap>) {
    Tout out{};
    for (unsigned int i = 0; i < inVal.size(); ++i)
      out[i] = wrapScalar(inVal[i], minVal[i], maxVal[i]);
    return out;
  }  
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

#endif /* ORG_EEROS_CONTROL_WRAPAROUND_HPP_ */
