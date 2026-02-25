#ifndef ORG_EEROS_CONTROL_RATELIMITER_HPP_
#define ORG_EEROS_CONTROL_RATELIMITER_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>


namespace eeros {
namespace control {

/**
 * The RateLimiter block limits the first derivative of the signal passing through it. 
 * The output changes no faster than the specified limit. 
 * The derivative is calculated using this equation: 
 * 		rate = (input(i)-output(i-1))/(time(i)-time(i-1))
 * 
 * The output is determined by comparing the rate of the signal to the rising 
 * slew rate and falling slew rate parameters:
 * If rate is greater than the rising slew rate parameter, the output is calculated as:
 * 		output(i) = dt * rRate + output(i-1)
 * If rate is less than the falling slew rate parameter, the output is calculated as:
 * 		output(i) = dt * fRate + output(i-1)
 * If rate is between the bounds of R and F, the change in output is equal to the change in input: 
 * 		output(i) = input(i)
 *
 * If the input is a vector, then it is possible to choose if the algorithm applies elementwise or not.
 * If yes, rRate and fRate must be vectors.
 * 
 * @tparam Tout - input and output signal data type (double - default type)
 * @tparam Trate - rate data type (double - default type)
 * @tparam Uin - input signal unit type (dimensionless - default type)
 * @tparam Uout - output signal unit type (dimensionless - default type)
 *
 * @since 1.1
 */

template< typename Tout = double, typename Trate = double, SIUnit Uin = SIUnit::create(), SIUnit Uout = SIUnit::create() >
class RateLimiter : public Blockio<1,1,Tout,Tout,MakeUnitArray<Uin>::value,MakeUnitArray<Uout>::value> {
 public:

  /**
   * Constructs a RateLimiter instance with rising and falling rates being equal.\n
   * 
   * @param rate - limit of the first derivative
   */
  RateLimiter(Trate rate) {
    fallingRate = -rate;
    risingRate = rate;
    outPrev.clear();
  }

  /**
   * Constructs a RateLimiter instance specifying rising and falling rates.\n
   * 
   * @param fRate - limit of the first derivative in negative direction
   * @param rRate - limit of the first derivative in positive direction
   */
  RateLimiter(Trate fRate, Trate rRate) {
    fallingRate = fRate;
    risingRate = rRate;
    outPrev.clear();
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  RateLimiter(const RateLimiter& s) = delete; 

  /**
   * Runs the rate limiting algorithm.
   * 
   * rate = (input(i)-output(i-1))/(time(i)-time(i-1))
   * 
   * If rate > risingRate: output(i) = dt * risingRate + output(i-1)
   * If rate < fallingRate: output(i) = dt * fallingRate + output(i-1)
   * Otherwise: output(i) = input(i)
   */
  void run() override {
    std::lock_guard<std::mutex> lock(mtx);
    Tout inVal = this->in.getSignal().getValue();
    double tin = this->in.getSignal().getTimestamp() / 1000000000.0;
    double tprev = outPrev.getTimestamp() / 1000000000.0;
    Tout outVal = inVal;
    if(enabled) {
      double dt = tin - tprev;
      outVal = calculateResult<Tout>(inVal, dt);
    }
    outPrev.setValue(outVal);
    outPrev.setTimestamp(this->in.getSignal().getTimestamp());
    this->out.getSignal().setValue(outVal);
    this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
  }

  /**
   * Enables the rate limiter block.
   * 
   * If enabled, run() will perform rate limit.
   * 
   * @see disable()
   */
  void enable() override {
    enabled = true;
  }

  /**
   * Disables the rate limiter block.
   * 
   * If disabled, run() will set output = input.
   * 
   * @see enable()
   */
  void disable() override {
    enabled = false;
  }

  /**
   * Sets falling and rising rates.
   * 
   * @param fRate - limit of the first derivative in negative direction
   * @param rRate - limit of the first derivative in positive direction
   */
  virtual void setRate(Trate fRate, Trate rRate) {
    std::lock_guard<std::mutex> lock(mtx);
    fallingRate = fRate;
    risingRate = rRate;
  }

  /*
   * Friend operator overload to give the operator overload outside
   * the class access to the private fields.
   */
  template <typename X>
  friend std::ostream& operator<<(std::ostream& os, RateLimiter<X>& rl);

 private:
  Signal<Tout> outPrev;
  Trate fallingRate, risingRate;
  bool enabled{false};
  std::mutex mtx;

  template <typename S> 
  typename std::enable_if<std::is_arithmetic<S>::value, S>::type calculateResult(S inValue, double dt) {
    Tout outVal;
    Trate rate = (inValue - outPrev.getValue()) / dt;
    if (rate > risingRate) outVal = dt * risingRate + outPrev.getValue();
    else if (rate < fallingRate) outVal = dt * fallingRate + outPrev.getValue();
    else outVal = inValue;
    return outVal;
  }

  template <typename S> 
  typename std::enable_if<std::is_compound<S>::value && std::is_arithmetic<Trate>::value, S>::type calculateResult(S inValue, double dt) {
    std::cout << " NOT element wise" << std::endl;
    Tout outVal;
    for (unsigned int i = 0; i < inValue.size(); i++) {
      double rate = (inValue[i] - outPrev.getValue()[i])/dt;
      if (rate > risingRate) outVal[i] = dt * risingRate + outPrev.getValue()[i];
      else if (rate < fallingRate) outVal[i] = dt * fallingRate + outPrev.getValue()[i];
      else outVal[i] = inValue[i];
    }
    return outVal;
  }

  template <typename S> 
  typename std::enable_if<std::is_compound<S>::value && std::is_compound<Trate>::value, S>::type calculateResult(S inValue, double dt) {
    std::cout << " element wise" << std::endl;
    Tout outVal;
    for (unsigned int i = 0; i < inValue.size(); i++) {
      double rate = (inValue[i] - outPrev.getValue()[i]) / dt;
      if (rate > risingRate[i]) outVal[i] = dt * risingRate[i] + outPrev.getValue()[i];
      else if (rate < fallingRate[i]) outVal[i] = dt * fallingRate[i] + outPrev.getValue()[i];
      else outVal[i] = inValue[i];
    }
    return outVal;
  }

};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * rate limiter instance to an output stream.\n
 * Does not print a newline control character.
 */
template< typename Tout, typename Trate, SIUnit Uin, SIUnit Uout >
std::ostream& operator<<(std::ostream& os, RateLimiter<Tout, Trate, Uin, Uout>& rl) {
  os << "Block RateLimiter: '" << rl.getName() << "' falling rate=" << rl.fallingRate << ", rising rate=" << rl.risingRate; 
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_RATELIMITER_HPP_ */
