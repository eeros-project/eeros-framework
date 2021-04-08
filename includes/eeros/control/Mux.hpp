#ifndef ORG_EEROS_CONTROL_MUX_HPP_
#define ORG_EEROS_CONTROL_MUX_HPP_

#include <eeros/control/Block1o.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/IndexOutOfBoundsFault.hpp>

namespace eeros {
namespace control {

/**
 * A multiplexer block is used to bundle multiple inputs into one output vector.
 *
 * @tparam N - number of inputs (double - default type)
 * @tparam T - signal type (double - default type)
 * @since v0.6
 */

template < uint32_t N, typename T = double, typename C = eeros::math::Matrix<N,1,T> >
class Mux: public Block1o<C> {
 public:
  /**
   * Constructs a multiplexer instance.
   */
  Mux() { 
    for(uint8_t i = 0; i < N; i++) in[i].setOwner(this);
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Mux(const Mux& s) = delete; 
  
  /**
   * Runs the multiplexer.
   *
   */
  virtual void run() {
    C newValue;
    for (uint32_t i = 0; i < N; i++) {
      newValue(i) = in[i].getSignal().getValue();
    }
    this->out.getSignal().setValue(newValue);
    this->out.getSignal().setTimestamp(in[0].getSignal().getTimestamp());
  }

  /**
   * Gets an input of the block.
   * 
   * @param index - index of an input
   * @return input
   */
  virtual Input<T>& getIn(uint32_t index) {
    if (index < 0 || index >= N)
      throw IndexOutOfBoundsFault("Trying to get inexistent input in block '" + this->getName() + "'");
    return in[index];
  }

 protected:
  Input<T> in[N];
};

}
}

#endif /* ORG_EEROS_CONTROL_MUX_HPP_ */
