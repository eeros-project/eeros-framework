#ifndef ORG_EEROS_CONTROL_KEYBOARDINPUT_HPP_
#define ORG_EEROS_CONTROL_KEYBOARDINPUT_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/Keyboard.hpp>
#include <eeros/hal/KeyboardDigIn.hpp>
#include <eeros/hal/KeyList.hpp>
#include <eeros/control/IndexOutOfBoundsFault.hpp>

using namespace eeros::hal;

namespace eeros {
namespace control {

/**
 * This block serves to read the keys from a keyboard. The state
 * of the keys are delivered as signals with values of type bool.
 *
 * @tparam N - number of input keys
 *
 * @since v0.6v
 */
template < uint8_t N >
class KeyboardInput: public Blockio<0,N,bool> {
 public:
  /**
   * Constructs a block which reads several keys on a keyboard.
   * 
   * @param asciiCode - ascii code of the key
   * @param name - name of the key
   * @param priority - priority of the associated thread
   */
  KeyboardInput(std::vector<char> asciiCode, std::vector<std::string> name, int priority = 20) : k(priority) {
    auto& list = KeyList::instance();
    list.addKeys(asciiCode, name);
    HAL& hal = HAL::instance();
    for (uint8_t i = 0; i < list.nofKeys; i++) {
      hal::Input<bool>* in = new KeyboardDigIn(list, list.key[i]);
      hal.addInput(in);
    }
  }

  /**
  * Disabling use of copy constructor because the block should never be copied unintentionally.
  */
  KeyboardInput(const KeyboardInput& s) = delete; 

  /**
   * Runs the block.
   */
  void run() override {
    uint64_t time = eeros::System::getTimeNs();
    auto& list = KeyList::instance();
    for (uint8_t i = 0; i < list.nofKeys; i++) {
      this->out[i].getSignal().setValue(list.state[i]);
      this->out[i].getSignal().setTimestamp(time);
    }
  }

  /**
   * Getter function for the output with a given index.
   * 
   * @param index - index of output
   * @return the output with this index
   */
  virtual Output<bool>& getOut(uint8_t index) {
    auto& list = KeyList::instance();
    if (index < 0 || index >= list.nofKeys) 
      throw IndexOutOfBoundsFault("Trying to get inexistent element of output in Block " + this->getName());  
    return this->out[index];
  }

  /**
   * Reset the state of a key. This must be done manually after having consumed the state.
   * 
   * @param index - index of output
   */
  virtual void reset(uint8_t index) {
    auto& list = KeyList::instance();
    if (index < 0 || index >= list.nofKeys) 
      throw IndexOutOfBoundsFault("Trying to get inexistent element of output in Block " + this->getName());  
    list.state[index] = false;
  }

 protected:
  Keyboard k;
};

/**
 * This block serves to read the keys from a keyboard. The state
 * of the keys are delivered as signals with values of type bool.
 * Spezialization for one key.
 *
 * @since v0.6v
 */
template < >
class KeyboardInput<1>: public Blockio<0,1,bool> {
 public:
  /**
   * Constructs a block which reads one key on a keyboard.
   * 
   * @param asciiCode - ascii code of the key
   * @param name - name of the key
   * @param priority - priority of the associated thread
   */
  KeyboardInput(std::vector<char> asciiCode, std::vector<std::string> name, int priority = 20) : k(priority) {
    auto& list = KeyList::instance();
    list.addKeys(asciiCode, name);
    HAL& hal = HAL::instance();
    hal::Input<bool>* in = new KeyboardDigIn(list, list.key[0]);
    hal.addInput(in);
  }

  /**
  * Disabling use of copy constructor because the block should never be copied unintentionally.
  */
  KeyboardInput(const KeyboardInput& s) = delete; 

  /**
   * Runs the block.
   */
  void run() override {
    uint64_t time = eeros::System::getTimeNs();
    auto& list = KeyList::instance();
    this->out.getSignal().setValue(list.state[0]);
    this->out.getSignal().setTimestamp(time);
  }

  /**
   * Reset the state of the key. This must be done manually after having consumed the state.
   */
  virtual void reset() {
    auto& list = KeyList::instance();
    list.state[0] = false;
  }

 protected:
  Keyboard k;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * KeyboardInput instance to an output stream.\n
 * Does not print a newline control character.
 */
template <uint8_t N>
std::ostream& operator<<(std::ostream& os, KeyboardInput<N>& k) {
  os << "Block keyboard input: '" << k.getName();
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_KEYBOARDINPUT_HPP_ */
