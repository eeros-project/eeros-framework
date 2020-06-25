#ifndef ORG_EEROS_CONTROL_SUM_HPP_
#define ORG_EEROS_CONTROL_SUM_HPP_

#include <eeros/control/Block1o.hpp>
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

template < uint8_t N = 2, typename T = double >
class Sum : public Block1o<T> {
 public:

  /**
   * Constructs a sum instance with all inputs to be added.\n
   */
  Sum() {
    for(uint8_t i = 0; i < N; i++) {
      negated[i] = false;
      in[i].setOwner(this);
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
    for(uint8_t i = 0; i < N; i++) {
      if(negated[i]) sum -= in[i].getSignal().getValue();
      else sum += in[i].getSignal().getValue();
    }
    this->out.getSignal().setValue(sum);
    this->out.getSignal().setTimestamp(in[0].getSignal().getTimestamp());
  }
  
  /**
  * Getter function for the input with a given index.
  * 
  * @param index - index of input
  * @return The input with this index
  */
  virtual Input<T>& getIn(uint8_t index) {
    if (index >= N) throw eeros::Fault("index too big in sum block '" + this->getName() + "'");
    return in[index];
  }
  
  /**
  * Allows to negate an input meaning the its signal is subtracted from the other input signals.
  * 
  * @param index - index of input
  */
  virtual void negateInput(uint8_t index) {
    if (index >= N) throw eeros::Fault("index too big in sum block '" + this->getName() + "'");
    negated[index] = true;
  }

 protected:
  Input<T> in[N];
  bool negated[N];
};

/********** Print functions **********/
template <uint8_t N, typename T>
std::ostream& operator<<(std::ostream& os, Sum<N,T>& sum) {
  os << "Block sum: '" << sum.getName() << "'"; 
  return os;
}

};
};

#endif /* ORG_EEROS_CONTROL_SUM_HPP_ */
