#ifndef ORG_EEROS_CONTROL_GenericBlock1i1o_HPP_
#define ORG_EEROS_CONTROL_GenericBlock1i1o_HPP_

#include <eeros/control/Block1i1o.hpp>
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
 * Define such a block as follows:
 * GenericBlock1i1o<Vector2> gen([&](){
 *   gen1.getOut().getSignal().setValue((gen1.getIn().getSignal().getValue() + 3.5 ) * 2.7 );
 *   gen1.getOut().getSignal().setTimestamp(gen1.getIn().getSignal().getTimestamp());
 * });
 *
 * @tparam T - value type (double - default type)
 *
 * @since v1.2.1
 */

template<typename T = double>
class GenericBlock1i1o : public Block1i1o<T> {
 public:

  /**
   * Constructs a GenericBlock1i1o instance which runs a given algorithm 
   * defined by the parameter function.
   *
   * @param f - function defining the algorithm
   */
  GenericBlock1i1o(std::function<void()> const &f) : func(f) { }

  /**
  * Disabling use of copy constructor because the block should never be copied unintentionally.
  */
  GenericBlock1i1o(const GenericBlock1i1o& s) = delete; 
  
  /**
   * Runs the generic algorithm.
   *
   */
  virtual void run() {
     func();
  }

  /*
   * Friend operator overload to give the operator overload outside
   * the class access to the private fields.
   */
  template<typename X>
  friend std::ostream &operator<<(std::ostream &os, GenericBlock1i1o<X> &gen);

 private:
  std::function<void()> func;
};


/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * GenericBlock1i1o instance to an output stream.\n
 * Does not print a newline control character.
 */
template<typename T>
std::ostream &operator<<(std::ostream &os, GenericBlock1i1o<T> &gen) {
  os << "Block GenericBlock1i1o: '" << gen.getName() << "'";
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_GenericBlock1i1o_HPP_ */
