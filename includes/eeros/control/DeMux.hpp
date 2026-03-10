#ifndef ORG_EEROS_CONTROL_DEMUX_HPP_
#define ORG_EEROS_CONTROL_DEMUX_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>

namespace eeros {
namespace control {
 
/**
 * A demultiplexer block is used to split an input vector 
 * to individual outputs.
 *
 * @tparam N - number of outputs
 * @tparam T - output signal data type (double - default type)
 * @tparam C - input signal data type (Matrix<N,1,T> - default type)
 * @tparam U - signal unit type (dimensionless - default type)
 *
 * @since v0.6
 */

template < uint32_t N, typename T = double, typename C = eeros::math::Matrix<N,1,T>, SIUnit U = SIUnit::create() >
class DeMux: public Blockio<1,N,C,T,MakeUnitArray<U>::value,MakeUnitArray<U,N>::value> {
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
  void run() override {
    for_<N>([&, this]<std::size_t I>() {
      this->template getOut<I>().getSignal().setValue(this->in.getSignal().getValue()(I));
      this->template getOut<I>().getSignal().setTimestamp(this->in.getSignal().getTimestamp());
    });
  }
      
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * Demultiplexer instance to an output stream.\n
 * Does not print a newline control character.
 */
template < uint32_t N, typename T, typename C, SIUnit U >
std::ostream& operator<<(std::ostream& os, const DeMux<N, T, C, U>& d) {
  os << "Block demultiplexer: '" << d.getName() << "'"; 
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_DEMUX_HPP_ */
