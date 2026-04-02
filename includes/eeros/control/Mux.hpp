#ifndef ORG_EEROS_CONTROL_MUX_HPP_
#define ORG_EEROS_CONTROL_MUX_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/IndexOutOfBoundsFault.hpp>

namespace eeros::control {

/**
 * A multiplexer block is used to bundle multiple inputs into one output vector.
 *
 * @tparam N - number of inputs
 * @tparam T - signal input type (double - default type)
 * 
 * @since v0.6
 */

template < uint32_t N, typename T = double >
class Mux: public Blockio<N,1,T,eeros::math::Matrix<N,1,T>> {
 public:
   
  /**
   * Constructs a multiplexer instance.
   */
  Mux() { }

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  Mux(const Mux& s) = delete;
  Mux& operator=(const Mux&) = delete;

  /**
   * Runs the multiplexer.
   *
   */
  virtual void run() {
    eeros::math::Matrix<N,1,T> newValue;
    for (uint32_t i = 0; i < N; i++) {
      newValue(i) = this->in[i].getSignal().getValue();
    }
    this->getOut().getSignal().setValue(newValue);
    this->getOut().getSignal().setTimestamp(this->in[0].getSignal().getTimestamp());
  }

};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * Multiplexer instance to an output stream.\n
 * Does not print a newline control character.
 */
template <uint8_t N, typename T>
std::ostream& operator<<(std::ostream& os, const Mux<N,T>& m) {
  os << "Block multiplexer: '" << m.getName() << "'"; 
  return os;
}

}

#endif /* ORG_EEROS_CONTROL_MUX_HPP_ */
