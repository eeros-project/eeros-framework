#ifndef ORG_EEROS_CONTROL_BLOCK1O_HPP_
#define ORG_EEROS_CONTROL_BLOCK1O_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Signal.hpp>

namespace eeros {
namespace control {

/**
 * Base class for all blocks with one output.
 * 
 * @tparam T - output type (double - default type)
 * @since v0.4
 */

template < typename T = double >
class Block1o : public Block {
 public:
  /**
   * Constructs an block with one output.
   */
  Block1o() : out(this) {
    this->out.getSignal().clear();
  }
        
  /**
   * Gets the output of the block.
   * 
   * @return output
   */
  virtual Output<T>& getOut() {
    return out;
  }
  
 protected:
  Output<T> out;
};

}
}

#endif /* ORG_EEROS_CONTROL_BLOCK1O_HPP_ */
