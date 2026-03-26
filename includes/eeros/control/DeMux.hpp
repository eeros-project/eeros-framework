#ifndef ORG_EEROS_CONTROL_DEMUX_HPP_
#define ORG_EEROS_CONTROL_DEMUX_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>

namespace eeros::control {
 
/**
 * A demultiplexer block is used to split an input vector 
 * to individual outputs.
 *
 * @tparam N - number of outputs
 * @tparam T - output signal data type (double - default type)
 * @tparam U - signal unit type (dimensionless - default type)
 *
 * @since v0.6
 */

template < uint32_t N, typename T = double, SIUnit U = SIUnit::create() >
class DeMux: public Blockio<1,N,eeros::math::Matrix<N,1,T>,T,MakeUnitArray<U>::value,MakeUnitArray<U,N>::value> {
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
template < uint32_t N, typename T, SIUnit U >
std::ostream& operator<<(std::ostream& os, const DeMux<N,T,U>& d) {
  os << "Block demultiplexer: '" << d.getName() << "'"; 
  return os;
}

}

#endif /* ORG_EEROS_CONTROL_DEMUX_HPP_ */
