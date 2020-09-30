#ifndef ORG_EEROS_CONTROL_DELAY_HPP_
#define ORG_EEROS_CONTROL_DELAY_HPP_

#include <eeros/control/Block1i1o.hpp>

namespace eeros {
namespace control {

/**
 * A delay block is used delay an input signal. The current input signal is stored
 * into a buffer while the output of the block is taken from the most last position 
 * of the delay buffer.
 *
 * @tparam T - signal type (double - default type)
 *
 * @since v1.2
 */

template < typename T = double >
class Delay : public Block1i1o<T> {
 public:
  /**
   * Constructs a delay block instance with a given delay in s.\n
   * The parameter delay together with the sampling time determine
   * the length of the buffer.
   * 
   * @param delay - delay in s
   * @param ts - sampling time in s
   */
  Delay(double delay, double ts) : delay(delay), bufLen(delay / ts), index(0), cycle(false) {
    if (bufLen < 1) throw eeros::Fault("delay has negative or zero length");
    buf = new T[bufLen];
    timeBuf = new timestamp_t[bufLen]; 
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Delay(const Delay& s) = delete; 

  /**
   * Destructor frees buffers.
   */
  ~Delay() {
    delete[] buf;
    delete[] timeBuf;
  }

  /**
   * Runs the delay block.   
   */
  virtual void run() {
    buf[index] = this->in.getSignal().getValue();
    timeBuf[index] = this->in.getSignal().getTimestamp();
    index++;
    if (index == bufLen) {
      index = 0;
      cycle = true;
    }
    if(cycle) {
      this->out.getSignal().setValue(buf[index]);
      this->out.getSignal().setTimestamp(timeBuf[index]);
    }
  }
      
  /*
   * Friend operator overload to give the operator overload outside
   * the class access to the private fields.
   */
  template <typename X>
  friend std::ostream& operator<<(std::ostream& os, Delay<X>& delay);
     
 protected:
  double delay; // delay in s
  uint32_t bufLen; // total size of buffer
  uint32_t index;   // current index
  bool cycle;   // indicates whether wrap around occured
  T* buf; // delay buffer for signal values
  timestamp_t* timeBuf; // delay buffer for timestamps
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * delay instance to an output stream.\n
 * Does not print a newline control character.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, Delay<T>& delay) {
  os << "Block delay: '" << delay.getName() << "' with a delay of " << delay.delay << "s"; 
  return os;
}

};
};

#endif /* ORG_EEROS_CONTROL_DELAY_HPP_ */
