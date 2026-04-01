#ifndef ORG_EEROS_CONTROL_MUL_HPP_
#define ORG_EEROS_CONTROL_MUL_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/control/Input.hpp>
#include <ostream>

namespace eeros::control {

/**
 * A multiplier block with two inputs and one output.
 *
 * Computes the product of two input signals and writes the result
 * to the output. The output timestamp is taken from input 1.
 *
 * Typical usage:
 * @code
 * Mul<double, double, double> mul;
 * mul.getIn1().connect(someOutput1);
 * mul.getIn2().connect(someOutput2);
 * @endcode
 *
 * @tparam In1T - type of the first  input signal (default: double)
 * @tparam In2T - type of the second input signal (default: double)
 * @tparam OutT - type of the output signal       (default: double)
 *
 * @since v0.6
 */
template < typename In1T = double, typename In2T = double, typename OutT = double >
class Mul : public Blockio<0,1,OutT> {
 public:
  /**
   * Constructs a Mul block and registers both inputs with this block as owner.
   */
  Mul() : in1(this), in2(this) { }

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  Mul(const Mul& s) = delete;
  Mul& operator=(const Mul&) = delete;

  /**
   * Runs the multiplication algorithm.
   *
   * Computes `out = in1 * in2` and sets the output timestamp
   * to the timestamp of input 1.
   */
  void run() override {
    OutT prod;
    prod = in1.getSignal().getValue() * in2.getSignal().getValue();
    this->getOut().getSignal().setValue(prod);
    this->getOut().getSignal().setTimestamp(in1.getSignal().getTimestamp());
  }
  
  /**
   * Returns the first input.
   *
   * @return reference to input 1
   */
  virtual Input<In1T>& getIn1() {
    return in1;
  }

  /**
   * Returns the second input.
   *
   * @return reference to input 2
   */
  virtual Input<In2T>& getIn2() {
    return in2;
  }

 protected:
  Input<In1T> in1;
  Input<In2T> in2;
};

/********** Print functions **********/
template <typename In1T = double, typename In2T = double, typename OutT = double>
std::ostream& operator<<(std::ostream& os, const Mul<In1T,In2T,OutT>& mul) {
  os << "Block multiplier: '" << mul.getName() << "'"; 
        return os;
}

}

#endif /* ORG_EEROS_CONTROL_MUL_HPP_ */
