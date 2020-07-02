#ifndef ORG_EEROS_CONTROL_STEP_HPP_
#define ORG_EEROS_CONTROL_STEP_HPP_

#include <eeros/control/Block1o.hpp>
#include <eeros/core/System.hpp>

namespace eeros {
namespace control {

/**
 * A step delivers a step function which switches the output value from a given initial value
 * to a new output value after a time delay has elapsed.
 * The reset function allows to repeat the step function.  
 * 
 * @tparam T - value type (double - default type)
 * 
 * @since v0.6
 */

template < typename T = double >
class Step : public Block1o<T> {
 public:
   
  /**
   * Constructs a default step instance with an initial value of 0.0, a step height of 1.0 and
   * a delay time of 0s.\n
   * Calls Step(T initValue, T stepHeight, double delayTime).
   *
   * @see Step(T initValue, T stepHeight, double delayTime)
   */
  Step() : Step(0.0, 1.0, 0) { }

  /**
   * Constructs a step instance with an initial value, a step height and a delay time.
   * After the delay time has elapsed, the output switches from its initial value to a 
   * value which is calculated by adding the step height to the initial value.\n
   *
   * @param initValue - initial output value
   * @param stepHeight - step height
   * @param delayTime - delay time
   */
  Step(T initValue, T stepHeight, double delayTime) : initValue(initValue), stepHeight(stepHeight), delayTime(delayTime) {
    first = true;
  }

  /**
  * Disabling use of copy constructor because the block should never be copied unintentionally.
  */
  Step(const Step& s) = delete; 
  
  /**
   * Runs the step algorithm.
   */
  virtual void run() {
    if(first) {
      time = delayTime + System::getTime();
      this->out.getSignal().setValue(initValue);
      first = false;
    }
    if(!stepDone && System::getTime() >= time) {
      this->out.getSignal().setValue(initValue + stepHeight);
      stepDone = true;
    }
    this->out.getSignal().setTimestamp(System::getTimeNs());
  }
  
  /**
   * The reset function sets the output to the initial value and resets the time, so that after the 
   * delay time the step function is repeated .\n
   */
  virtual void reset() {
    stepDone = false;
    first = true;
  }
  
  /**
   * Sets the initial value.\n
   *
   * @param initValue - initial output value
   */
  virtual void setInitValue(T initValue) {
    this->initValue = initValue;
  }
  
  /**
   * Sets the step height.\n
   *
   * @param stepHeight - step height
   */
  virtual void setStepHeight(T stepHeight) {
    this->stepHeight = stepHeight;
  }
  
  /**
   * Sets the delay time.\n
   *
   * @param delayTime - delay time
   */
  virtual void setDelayTime(double delayTime) {
    this->delayTime = delayTime;
  }

  /*
   * Friend operator overload to give the operator overload outside
   * the class access to the private fields.
   */
  template <typename X>
  friend std::ostream& operator<<(std::ostream& os, Step<X>& step);
  
protected:
  T initValue;
  T stepHeight;
  double delayTime, time;
  bool stepDone;
  bool first;
};

/********** Print functions **********/
template <typename T>
std::ostream& operator<<(std::ostream& os, Step<T>& step) {
  os << "Block step: '" << step.getName() << "' init val = " << step.initValue << ", step height = " << step.stepHeight << ", delay = " << step.delayTime; 
  return os;
}

};
};

#endif /* ORG_EEROS_CONTROL_STEP_HPP_ */
