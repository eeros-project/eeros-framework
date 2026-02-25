#ifndef ORG_EEROS_CONTROL_SUM_HPP_
#define ORG_EEROS_CONTROL_SUM_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/control/Input.hpp>

namespace eeros {
namespace control {

/**
 * A sum allows to add the signals of two or more inputs together.
 * Any of the inputs can be inverted which allows to not only adding but also subtracting signals.
 * 
 * @tparam N - number of inputs
 * @tparam T - value type (double - default type)
 * 
 * @since v0.4
 */

template < uint8_t N = 2, typename T = double, std::array<SIUnit, N> Uin = SIUnit::generateNSizeArray<N>(), SIUnit Uout = SIUnit::create() >
class Sum : public Blockio<N,1,T, T, Uin, MakeUnitArray<Uout>::value> {
 public:

  /**
   * Constructs a sum instance with all inputs to be added.\n
   */
  Sum() : first(true) {
    for(uint8_t i = 0; i < N; i++) {
      negated[i] = false;
      init[i] = false;
    }
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  Sum(const Sum& s) = delete; 

  /**
   * Runs the sum block.
   */
  virtual void run() {
    T sum; sum = 0; // TODO works only with primitive types or eeros::math::Matrix -> make specialization and use fill() for compatibility with std::array;
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
  virtual void negateInput(uint8_t index) {
    if (index >= N) throw eeros::Fault("Trying to get inexistent element of input vector in block '" + this->getName() + "'");
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
  virtual void setInitCondition(uint8_t index, T val) {
    if (index >= N) throw eeros::Fault("Trying to get inexistent element of input vector in block '" + this->getName() + "'");
    init[index] = true;
    initVal[index] = val;
  }


 private:
  bool negated[N];
  bool first;
  bool init[N];
  T initVal[N];
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * Sum instance to an output stream.\n
 * Does not print a newline control character.
 */
template <uint8_t N, typename T, std::array<SIUnit, static_cast<std::size_t>(N)> Uin, SIUnit Uout>
std::ostream& operator<<(std::ostream& os, Sum<N,T, Uin, Uout>& sum) {
  os << "Block sum: '" << sum.getName() << "'"; 
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_SUM_HPP_ */
