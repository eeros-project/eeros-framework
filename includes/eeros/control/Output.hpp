#ifndef ORG_EEROS_CONTROL_OUTPUT_HPP_
#define ORG_EEROS_CONTROL_OUTPUT_HPP_

#include <eeros/SIUnit.hpp>
#include <eeros/control/Signal.hpp>
#include <eeros/control/Block.hpp>

namespace eeros {
namespace control {

/**
 * Blocks can have inputs and outputs. This is the output class.
 * An output carries a signal. One or several inputs of other blocks
 * can be connected to this output.
 * 
 * @tparam T - signal data type (double - default type)
 * @tparam Unit - signal unit type (dimensionless - default type)
 * @since v0.4
 */

template < typename T = double, SIUnit Unit = SIUnit::create()   >
class Output {
 public:
  /**
   * Constructs an output instance.
   */
  Output() : owner(nullptr) { }

  /**
   * Constructs an output instance.
   *
   * @param owner - the block which owns this output
   */
  Output(Block* owner) : owner(owner) { }

  /**
   * Returns the signal which is carried by this output.
   * 
   * @return signal 
   */
  virtual Signal<T>& getSignal() {
    return signal;
  }

  /**
   * Every output is owned by a block. Sets the owner of this output.
   * 
   * @param block - owner of this output
   */
  virtual void setOwner(Block* block) {
    owner = block;
  }

 private:
  Signal<T> signal;
  Block* owner;
};

}
}

#endif /* ORG_EEROS_CONTROL_OUTPUT_HPP_ */
