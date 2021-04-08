#ifndef ORG_EEROS_CONTROL_DEMUX_HPP_
#define ORG_EEROS_CONTROL_DEMUX_HPP_

#include <eeros/control/Block1i.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/IndexOutOfBoundsFault.hpp>

namespace eeros {
namespace control {
    
/**
 * A demultiplexer block is used to split an input vector 
 * to individual outputs.
 *
 * @tparam N - number of outputs (double - default type)
 * @tparam T - signal type (double - default type)
 * @since v0.6
 */

template < uint32_t N, typename T = double, typename C = eeros::math::Matrix<N,1,T> >
class DeMux: public Block1i<C> {
 public:
  /**
   * Constructs a demultiplexer instance.
   */
  DeMux() { 
    for(uint32_t i = 0; i < N; i++) {
      this->out[i].getSignal().clear();
      out[i].setOwner(this);
    }
  }
      
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  DeMux(const DeMux& s) = delete; 

  /**
   * Runs the demultiplexer.
   *
   */
  virtual void run() {
    for(uint32_t i = 0; i < N; i++) {
      out[i].getSignal().setValue(this->in.getSignal().getValue()(i));
      out[i].getSignal().setTimestamp(this->in.getSignal().getTimestamp());
    }
  }
      
  /**
   * Gets an output of the block.
   * 
   * @param index - index of an output
   * @return output
   */
  virtual Output<T>& getOut(uint32_t index) {
    if (index < 0 || index >= N)
      throw IndexOutOfBoundsFault("Trying to get inexistent output in block '" + this->getName() + "'");
    return out[index];
  }
      
 protected:
  Output<T> out[N];
};

}
}

#endif /* ORG_EEROS_CONTROL_DEMUX_HPP_ */
