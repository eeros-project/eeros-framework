#ifndef ORG_EEROS_CONTROL_DEMUX_HPP_
#define ORG_EEROS_CONTROL_DEMUX_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/IndexOutOfBoundsFault.hpp>

namespace eeros {
namespace control {
    
/**
 * A demultiplexer block is used to split an input vector 
 * to individual outputs.
 *
 * @tparam N - number of outputs
 * @tparam T - signal type (double - default type)
 * @since v0.6
 */

template < uint32_t N, typename T = double, typename C = eeros::math::Matrix<N,1,T> >
class DeMux: public Blockio<1,N,C,T> {
 public:
  /**
   * Constructs a demultiplexer instance.
   */
  DeMux() { }
      
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
      this->out[i].getSignal().setValue(this->in.getSignal().getValue()(i));
      this->out[i].getSignal().setTimestamp(this->in.getSignal().getTimestamp());
    }
  }
      
};

}
}

#endif /* ORG_EEROS_CONTROL_DEMUX_HPP_ */
