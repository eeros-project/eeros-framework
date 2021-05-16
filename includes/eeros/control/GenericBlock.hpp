#ifndef ORG_EEROS_CONTROL_GenericBlock_HPP_
#define ORG_EEROS_CONTROL_GenericBlock_HPP_

#include <eeros/control/Block.hpp>
#include <functional>

namespace eeros {
namespace control {

/**
 * This blocks runs a given algorithm which can be set when creating the 
 * block. It has exactly one input and one output of the same type.
 * 
 * Such a block can be used when the algorithm is simple and one 
 * wants to avoid using several other blocks doing a simple algorithm, e.g. 
 * adding a offset and scale to a signal.
 * 
 * Define such a block with an example algorithm as follows:
 * GenericBlock<2,1,Vector2,Vector2> gen([&]() {
 *   auto val = (gen.getIn(0).getSignal().getValue() + 0.5) * 2;
 *   val[0] *= -1.0;
 *   val += gen.getIn(1).getSignal().getValue() + 1.0;
 *   gen.getOut(0).getSignal().setValue(val);
 *   gen.getOut(0).getSignal().setTimestamp(gen.getIn(0).getSignal().getTimestamp());
 * });
 *
 * @tparam N - number of inputs (1 - default)
 * @tparam M - number of outputs (1 - default)
 * @tparam Tin - input type (double - default type)
 * @tparam Tout - output type (double - default type)
 *
 * @since v1.2.1
 */

template<uint8_t N = 1, uint8_t M = 1, typename Tin = double, typename Tout = Tin>
class GenericBlock : public Block {
 public:

  /**
   * Constructs a GenericBlock1i1o instance which runs a given algorithm 
   * defined by the parameter function.
   *
   * @param f - function defining the algorithm
   */
  GenericBlock(std::function<void()> const &f) : func(f) { }

  /**
  * Disabling use of copy constructor because the block should never be copied unintentionally.
  */
  GenericBlock(const GenericBlock& s) = delete; 
  
  /**
   * Runs the generic algorithm.
   *
   */
  virtual void run() {
     func();
  }

  /**
   * Gets one of the inputs of the block.
   * 
   * @return input
   */
  virtual Input<Tin>& getIn(uint8_t index) {
    if (index >= N) throw eeros::Fault("index too big in generic block '" + this->getName() + "'");
    return in[index];
  }

  /**
   * Gets one of the outputs of the block.
   * 
   * @return output
   */
  virtual Output<Tout>& getOut(uint8_t index) {
    if (index >= N) throw eeros::Fault("index too big in generic block '" + this->getName() + "'");
    return out[index];
  }

  /*
   * Friend operator overload to give the operator overload outside
   * the class access to the private fields.
   */
  template<uint8_t U, uint8_t V, typename X, typename Y>
  friend std::ostream &operator<<(std::ostream &os, GenericBlock<U,V,X,Y> &gen);

 protected:
  Input<Tin> in[N];
  Output<Tout> out[N];

 private:
  std::function<void()> func;
};


/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * GenericBlock1i1o instance to an output stream.\n
 * Does not print a newline control character.
 */
template<uint8_t N = 1, uint8_t M = 1, typename Tin = double, typename Tout = Tin>
std::ostream &operator<<(std::ostream &os, GenericBlock<N,M,Tin,Tout> &gen) {
  os << "Block GenericBlock: '" << gen.getName() << "'";
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_GenericBlock_HPP_ */
