#ifndef ORG_EEROS_CONTROL_INPUTSUB_HPP_
#define ORG_EEROS_CONTROL_INPUTSUB_HPP_

#include <eeros/control/NotConnectedFault.hpp>
#include <eeros/control/Signal.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Block.hpp>

namespace eeros {
namespace control {

/**
 * An input of a subsystem must have all properties of a regular input.
 * Further, the internal blocks of the subsystem might be connected to
 * such an input. For this purpose this input must be able to be connected to.
 * 
 * @tparam T - signal type (double - default type)
 * @since v1.2.1
 */

template < typename T = double >
class InputSub : public Input<T>, public Output<T> {
 public:
  /**
   * Constructs an input instance for a subsystem.
   */
  InputSub() { }
 
  /**
   * Constructs an input instance for a subsystem.
   *
   * @param owner - the block which owns this input
   */
  InputSub(Block* owner) : Input<T>(owner) { }
          
  /**
   * Returns the signal which is carried by the output to which
   * this input is connected. If the input is not connected an NotConnectedFault
   * is thrown.
   * 
   * @return signal 
   */
  virtual Signal<T>& getSignal() {
    return Input<T>::getSignal();
  }
            
 };

}
}

#endif /* ORG_EEROS_CONTROL_INPUTSUB_HPP_ */
