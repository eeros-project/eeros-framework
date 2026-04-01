#ifndef ORG_EEROS_CONTROL_D_HPP_
#define ORG_EEROS_CONTROL_D_HPP_

#include <eeros/control/Blockio.hpp>

namespace eeros::control {

/**
 * An differentiator block is used to differentiate an input signal. 
 *
 * @tparam T - input and output signal data type (double - default type)
 *
 * @since v1.0
 */
template < typename T = double >
class D: public Blockio<1,1,T> {
 public:
  /**
   * Constructs an differentiator instance.
   */
  D() { prev.clear(); }
  
  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  D(const D& s) = delete;
  D& operator=(const D&) = delete;

  /**
   * Runs the differentiator algorithm. If the input signal
   * carries the same time stamp as the last sampled input signal
   * the output value and time stamp are left unchanged.
   * After the first run, the block would still carry a nan, due to 
   * its memory. Therefore, the output will be set to zero.
   */
  void run() override {
    Signal<T> sig = this->getIn().getSignal();
    double tin = sig.getTimestamp() / 1000000000.0;
    double tprev = prev.getTimestamp() / 1000000000.0;
    T valin = sig.getValue();
    T valprev = prev.getValue();
      
    if (first) {
      prev = this->getIn().getSignal();
      valOut = 0;
      timeOut = sig.getTimestamp();
      first = false;
    } else {
      if (tin != tprev) {
        valOut = (valin - valprev) / (tin - tprev);
        timeOut = (sig.getTimestamp() + prev.getTimestamp()) / 2;
      }
    }
      
    this->getOut().getSignal().setValue(valOut);
    this->getOut().getSignal().setTimestamp(timeOut);
    prev = sig;
  }
  
 private:
  Signal<T> prev;
  T valOut;
  bool first = true;
  timestamp_t timeOut;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * Differentiator instance to an output stream.\n
 * Does not print a newline control character.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const D<T>& d) {
  os << "Block differentiator: '" << d.getName() << "'";
  return os;
}

}

#endif /* ORG_EEROS_CONTROL_D_HPP_ */
