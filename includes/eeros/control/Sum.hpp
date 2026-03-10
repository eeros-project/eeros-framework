#ifndef ORG_EEROS_CONTROL_SUM_HPP_
#define ORG_EEROS_CONTROL_SUM_HPP_

#include <eeros/control/Blockio.hpp>

namespace eeros {
namespace control {

/**
 * A sum allows to add the signals of two or more inputs with the same SIUnit together,
 * while the SIUnit of the output must be the same.
 * Any of the inputs can be inverted which allows to not only adding but also subtracting signals.
 * 
 * @tparam N - number of inputs
 * @tparam T - value type (double - default type)
 * @tparam U - signal unit type (dimensionless - default type)
 * 
 * @since v0.4
 */

template < uint8_t N = 2, typename T = double, SIUnit U = SIUnit::create() >
class Sum : public Blockio<N,1,T,T,MakeUnitArray<U,N>::value,MakeUnitArray<U>::value> {
 public:

  /**
   * Constructs a sum instance with all inputs to be added.\n
   */
  Sum() : first(true), negated{}, init{} { }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Sum(const Sum& s) = delete; 

  /**
   * Runs the sum block.
   */
  void run() override {
    T sum{};
    if (first) {
      for_<N>([&, this]<std::size_t I>() {
        T val;
        if (init[I]) val = initVal[I];
        else val = this->template getIn<I>().getSignal().getValue();
        if (negated[I]) sum -= val;
        else sum += val;
      });
      first = false;
    } else {
      for_<N>([&, this]<std::size_t I>() {
        if (negated[I]) sum -= this->template getIn<I>().getSignal().getValue();
        else sum += this->template getIn<I>().getSignal().getValue();
      });
    }
    this->out.getSignal().setValue(sum);
    this->out.getSignal().setTimestamp(this->template getIn<0>().getSignal().getTimestamp());
  }

  /**
   * Allows to negate an input meaning the its signal is subtracted from the other input signals.
   * 
   * @param index - index of input
   */
  constexpr void negateInput(uint8_t index) {
    if (index >= N)
      throw eeros::Fault("Trying to get inexistent element of input vector in block '" + this->getName() + "'");
    negated[index] = true;
  }
  
  /**
   * Set the initial state of a given input to a sum block. This allows to determine an initial state 
   * where this initial state would be a nan value in case of a feedback path. This can be helpful when 
   * the input signal comes from a block which is later in the chain and has not run yet 
   * therefore delivering a nan signal.
   *
   * @see enable()
   * @param index - index of input
   * @param val - initial state
   */
  constexpr void setInitCondition(uint8_t index, T val) {
    if (index >= N)
      throw eeros::Fault("Trying to get inexistent element of input vector in block '" + this->getName() + "'");
    init[index] = true;
    initVal[index] = val;
  }


 private:
  bool first;
  std::array<bool,N> negated;
  std::array<bool,N> init;
  std::array<T,N> initVal;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * Sum instance to an output stream.\n
 * Does not print a newline control character.
 */
template <uint8_t N, typename T, SIUnit U>
std::ostream& operator<<(std::ostream& os, const Sum<N,T,U>& sum) {
  os << "Block sum: '" << sum.getName() << "'"; 
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_SUM_HPP_ */
