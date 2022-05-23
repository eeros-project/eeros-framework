#ifndef ORG_EEROS_CONTROL_BLOCK1I_HPP_
#define ORG_EEROS_CONTROL_BLOCK1I_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Signal.hpp>

namespace eeros {
namespace control {
        
/**
 * Base class for all blocks with one input.
 * 
 * @tparam T - input type (double - default type)
 * @since v0.4
 * @deprecated Will be removed in future releases
 */

template < typename T = double >
class [[deprecated("Replaced by Blockio<1,0>")]] Block1i : public Block {
 public:
  /**
   * Constructs an block with one input.
   */
  Block1i() : in(this) { }

  /**
   * Gets the input of the block.
   * 
   * @return input
   */
  virtual Input<T>& getIn() {
    return in;
  }

 protected:
  Input<T> in;
};

}
}

#endif /* ORG_EEROS_CONTROL_BLOCK1I_HPP_ */
