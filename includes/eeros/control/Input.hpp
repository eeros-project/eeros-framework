#ifndef ORG_EEROS_CONTROL_INPUT_HPP_
#define ORG_EEROS_CONTROL_INPUT_HPP_

#include <eeros/control/NotConnectedFault.hpp>
#include <eeros/control/Signal.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Block.hpp>

namespace eeros {
namespace control {

/**
 * Blocks can have inputs and outputs. This is the input class.
 * An input can be connected to an output of another block.
 * 
 * @tparam T - signal type (double - default type)
 * @since v0.4
 */

template < typename T = double >
class Input {
 public:
  /**
   * Constructs an input instance.
   */
  Input() : connectedOutput(nullptr), owner(nullptr) { }
 
  /**
   * Constructs an input instance.
   *
   * @param owner - the block which owns this input
   */
  Input(Block* owner) : connectedOutput(nullptr), owner(owner) { }

  /**
   * Connects an existing output of any other block to this input.
   * 
   * @param output - output of another block
   * @return true, if connection could be made 
   */
  virtual bool connect(Output<T>& output) {
    if(connectedOutput != nullptr) return false;
    connectedOutput = &output;
    return true;
  }
            
  /**
   * Connects an existing output of any other block to this input.
   * 
   * @param output - output of another block
   * @return true, if connection could be made 
   */
  virtual bool connect(Output<T>* output) {
    if(connectedOutput != nullptr) return false;
    connectedOutput = output;
    return true;
  }

  /**
   * Disconnects this input.
   */
  virtual void disconnect() {
    connectedOutput = nullptr;
  }

  /**
   * Queries the connection state of this input.
   * 
   * @return true, if connection exists to output of another block 
   */
  virtual bool isConnected() const {
    return connectedOutput != nullptr;
  }
            
  /**
   * Returns the signal which is carried by the output to which
   * this input is connected. If the input is not connected an NotConnectedFault
   * is thrown.
   * 
   * @return signal 
   */
  virtual Signal<T>& getSignal() {
    if(isConnected()) return connectedOutput->getSignal();
    std::string name;
    if (owner != nullptr) name = owner->getName(); else name = "";
      throw NotConnectedFault("Read from an unconnected input in block '" + name + "'");
  }
            
  /**
   * Every input is owned by a block. Sets the owner of this input.
   * 
   * @param block - owner of this input
   */
  virtual void setOwner(Block* block) {
    owner = block;
  }

  /**
   * Return the connected output
   * If the input is not connected an NotConnectedFault
   *
   * @return Output
   */
  virtual Output<T>& getConOutput() {
    if(isConnected()) return *connectedOutput;
    std::string name;
    if (owner != nullptr) name = owner->getName(); else name = "";
      throw NotConnectedFault("Tried to get an unconnnected block '" + name + "'");
  }
            
 protected:
  Output<T>* connectedOutput;
  Block* owner;
 };

}
}

#endif /* ORG_EEROS_CONTROL_INPUT_HPP_ */
