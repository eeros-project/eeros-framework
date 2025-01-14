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
 * @tparam T - signal output type (double - default type)
 * @tparam C - signal input type (Matrix<N,1,T> - default type)
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

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * Demultiplexer instance to an output stream.\n
 * Does not print a newline control character.
 */
template <uint8_t N, typename T, typename C>
std::ostream& operator<<(std::ostream& os, DeMux<N,T,C>& d) {
  os << "Block demultiplexer: '" << d.getName() << "'"; 
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_DEMUX_HPP_ */
