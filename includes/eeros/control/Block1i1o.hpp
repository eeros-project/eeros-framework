#ifndef ORG_EEROS_CONTROL_BLOCK1I1O_HPP_
#define ORG_EEROS_CONTROL_BLOCK1I1O_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>

namespace eeros {
namespace control {

/**
 * Base class for all blocks with one input and one output.
 * 
 * @tparam Tin - input type (double - default type)
 * @tparam Tout - output type (double - default type)
 * @since v0.4
 */

template < typename Tin = double, typename Tout = Tin >
class Block1i1o : public Block {
 public:
  /**
   * Constructs an block with one input and one output.
   */
  Block1i1o() : in(this), out(this) {
    this->out.getSignal().clear();
  }

  /**
   * Gets the input of the block.
   * 
   * @return input
   */
  virtual Input<Tin>& getIn() {
    return in;
  }

  /**
   * Gets the output of the block.
   * 
   * @return output
   */
  virtual Output<Tout>& getOut() {
    return out;
  }

 protected:
  Input<Tin> in;
  Output<Tout> out;
};

}
}
#endif /* ORG_EEROS_CONTROL_BLOCK1I1O_HPP_ */
