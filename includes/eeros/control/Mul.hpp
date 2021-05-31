#ifndef ORG_EEROS_CONTROL_MUL_HPP_
#define ORG_EEROS_CONTROL_MUL_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/control/Input.hpp>

namespace eeros {
namespace control {

template < typename In1T = double, typename In2T = double, typename OutT = double >
class Mul : public Blockio<0,1,OutT> {
 public:
  Mul() : in1(this), in2(this) { }

  virtual void run() {
    OutT prod;
    prod = in1.getSignal().getValue() * in2.getSignal().getValue();
    this->out.getSignal().setValue(prod);
    this->out.getSignal().setTimestamp(in1.getSignal().getTimestamp());
  }
  
  virtual Input<In1T>& getIn1() {
    return in1;
  }

  virtual Input<In2T>& getIn2() {
    return in2;
  }

 protected:
  Input<In1T> in1;
  Input<In2T> in2;
};

/********** Print functions **********/
template <typename In1T = double, typename In2T = double, typename OutT = double>
std::ostream& operator<<(std::ostream& os, Mul<In1T,In2T,OutT>& mul) {
  os << "Block multiplier: '" << mul.getName() << "'"; 
        return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_MUL_HPP_ */
