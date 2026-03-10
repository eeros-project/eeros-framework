#ifndef ORG_EEROS_CONTROL_MUX_HPP_
#define ORG_EEROS_CONTROL_MUX_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>

namespace eeros {
namespace control {

/**
 * A multiplexer block is used to bundle multiple inputs into one output vector.
 *
 * @tparam N - number of inputs
 * @tparam T - input signal data type (double - default type)
 * @tparam C - output signal data type (Matrix<N,1,T> - default type)
 * @tparam U - signal unit type (dimensionless - default type)
 *
 * @since v0.6
 */

template < uint32_t N, typename T = double, typename C = eeros::math::Matrix<N,1,T>, SIUnit U = SIUnit::create() >
class Mux: public Blockio<N,1,T,C,MakeUnitArray<U,N>::value,MakeUnitArray<U>::value> {
 public:
  /**
   * Constructs a multiplexer instance.
   */
  Mux() { }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Mux(const Mux& s) = delete; 

  /**
   * Runs the multiplexer.
   *
   */
  void run() override {
    C newValue;
    for_<N>([&, this]<std::size_t I>() {
      newValue(I) = this->template getIn<I>().getSignal().getValue();
    });
    this->out.getSignal().setValue(newValue);
    this->out.getSignal().setTimestamp(this->template getIn<0>().getSignal().getTimestamp());
  }

};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * Multiplexer instance to an output stream.\n
 * Does not print a newline control character.
 */
template < uint32_t N, typename T, typename C, SIUnit U >
std::ostream& operator<<(std::ostream& os, const Mux<N, T, C, U>& m) {
  os << "Block multiplexer: '" << m.getName() << "'"; 
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_MUX_HPP_ */
