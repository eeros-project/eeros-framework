#ifndef ORG_EEROS_CONTROL_FILTER_LOWPASSFILTER_HPP
#define ORG_EEROS_CONTROL_FILTER_LOWPASSFILTER_HPP

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>

namespace eeros {
namespace control {

/**
 * A low pass filter block is used to filter an input signal. 
 * The output signal value depends on the current and the past
 * input signal value, according to this equation:
 * valOut = valin*alpha + valprev*(1-alpha) 
 * 
 * The filter can be disabled. In this case the current input will be
 * directed to the output.
 * 
 * LowPassFilter is a class template with one type template argument.
 * The type template argument specifies the type which is used for the 
 * values when the class template is instantiated.
 * 
 * If the LowPassFilter is used with matrices (Matrix, Vector), the filter algorithm
 * will consider all values in the matrice and will not separate them.
 * For example a 3-tuple of a Vector3 instance will be kept together during processing
 * in the LowPassFilter.\n
 * 
 * @tparam T - value type (double - default type)
 * 
 * @since 1.3
 */

template<typename T = double>
class LowPassFilter : public Blockio<1,1,T> {
 public:
   
  /**
   * Constructs a default low pass filter.
   *
   * @param alpha - weight of input in the filter (value between 0 and 1)
   */
  LowPassFilter(double alpha) : alpha(alpha) { 
    prev.clear();
  }
    
  /**
  * Disabling use of copy constructor because the block should never be copied unintentionally.
  */
  LowPassFilter(const LowPassFilter& s) = delete; 
  
  /**
   * Runs the filter algorithm.
   *
   * Reads input data and calculates output depending on alpha param, input and previous output:
   * out = alpha*in + (1-alpha)*out_prev
   * Saves output for next run
   */
  virtual void run() override {
    Signal<T> sig = this->in.getSignal(); 
    T valin = sig.getValue();
    T valprev = prev.getValue();
    if (first) {
      valprev = valin;
      first = false;
    }
    T valOut;
    if(enabled) valOut = valin * alpha + valprev * (1 - alpha); 
    else valOut = valin;
    this->out.getSignal().setValue(valOut);
    this->out.getSignal().setTimestamp(sig.getTimestamp());
    prev = this->out.getSignal();
  }
  
  /**
   * Enables the filter.
   *
   * If enabled, run() will set the output signal to the filtered value
   *
   * @see run()
   */
  virtual void enable() override {
    enabled = true;
  }

  /**
   * Disables the filter.
   *
   * If disabled, run() will set the output signal to the input signal.
   *
   * @see run()
   */
  virtual void disable() override {
    enabled = false;
  }

  /*
   * Friend operator overload to give the operator overload outside 
   * the class access to the private fields.
   */
  template <typename ValT>
  friend std::ostream& operator<<(std::ostream& os, LowPassFilter<ValT>& filter);

 private:
  double alpha{1.0};
  bool first{true};
  bool enabled{true};
  Signal<T> prev;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * LowPassFilter instance to an output stream.
 * Does not print a newline control character.
 */
template <typename Tval>
std::ostream& operator<<(std::ostream& os, LowPassFilter<Tval>& filter) {
  os << "Block LowPassFilter: '" << filter.getName() << "' is enabled=";
  os << filter.enabled << ", ";
  os << "alpha=" << filter.alpha;
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_FILTER_LOWPASSFILTER_HPP */
