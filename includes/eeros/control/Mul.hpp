#ifndef ORG_EEROS_CONTROL_MUL_HPP_
#define ORG_EEROS_CONTROL_MUL_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/control/Input.hpp>

namespace eeros {
namespace control {

/**
 * A multiplier block is used to combine two input values into one output value by multiplying them.
 *
 * @tparam In1T - first signal input data type (double - default type)
 * @tparam In2T - second signal input data type (double - default type)
 * @tparam OutT - output signal data type (Matrix<N,1,T> - default type)
 * @tparam Uin - input signal unit type (dimensionless - default type)
 * @tparam Uout - output signal unit type (dimensionless - default type)
 * @since v0.6
 */
template < typename In1T = double, typename In2T = double, typename OutT = double, std::array<SIUnit, 2> Uin = SIUnit::generateNSizeArray<2>(), SIUnit Uout = SIUnit::create() >
class Mul : public Blockio<0,1,OutT,OutT,SIUnit::generateNSizeArray<0>(),MakeUnitArray<Uout>::value> {
 public:
  Mul() : in1(this), in2(this) { }

  virtual void run() {
    OutT prod;
    prod = in1.getSignal().getValue() * in2.getSignal().getValue();
    this->out.getSignal().setValue(prod);
    this->out.getSignal().setTimestamp(in1.getSignal().getTimestamp());
  }

  virtual Input<In1T, Uin[0]>& getIn1() {
    return in1;
  }

  virtual Input<In2T, Uin[1]>& getIn2() {
    return in2;
  }

 protected:
  Input<In1T, Uin[0]> in1;
  Input<In2T, Uin[1]> in2;
};

/********** Print functions **********/
template <typename In1T = double, typename In2T = double, typename OutT = double, std::array<SIUnit, 2> Uin = SIUnit::generateNSizeArray<2>(), SIUnit Uout = SIUnit::create()>
std::ostream& operator<<(std::ostream& os, Mul<In1T,In2T,OutT, Uin, Uout>& mul) {
  os << "Block multiplier: '" << mul.getName() << "'"; 
        return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_MUL_HPP_ */
