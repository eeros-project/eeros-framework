#ifndef ORG_EEROS_CONTROL_D_HPP_
#define ORG_EEROS_CONTROL_D_HPP_

#include <eeros/control/Block1i1o.hpp>

namespace eeros {
namespace control {

/**
 * An differentiator block is used to differentiate an input signal. 
 *
 * @tparam T - output type (double - default type)
 * @since v1.0
 */
template < typename T = double >
class D: public Block1i1o<T> {
 public:
  /**
   * Constructs an differentiator instance.
   */
  D() {
    this->out.getSignal().clear();
    prev.clear();
  }
  
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  D(const D& s) = delete; 

  /**
   * Runs the differentiator algorithm. If the input signal
   * carries the same time stamp as the last sampled input signal
   * the output value and time stamp are left unchanged.
   */
  virtual void run() {
    Signal<T> sig = this->in.getSignal(); 
    double tin = sig.getTimestamp() / 1000000000.0;
    double tprev = prev.getTimestamp() / 1000000000.0;
    T valin = sig.getValue();
    T valprev = prev.getValue();
      
    if (tin != tprev) {
      valOut = (valin - valprev) / (tin - tprev);
      timeOut = (sig.getTimestamp() + prev.getTimestamp()) / 2;
    }
      
    this->out.getSignal().setValue(valOut);
    this->out.getSignal().setTimestamp(timeOut);
    prev = sig;
  }
  
  /*
   * Friend operator overload to give the operator overload outside
   * the class access to the private fields.
   */
  template<typename X>
  friend std::ostream &operator<<(std::ostream &os, D<X> &d);

 private:
  Signal<T> prev;
  T valOut;
  timestamp_t timeOut;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * Differentiator instance to an output stream.\n
 * Does not print a newline control character.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, D<T>& d) {
  os << "Block differentiator: '" << d.getName();
  return os;
}

}
}
#endif /* ORG_EEROS_CONTROL_D_HPP_ */
