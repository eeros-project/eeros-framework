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

namespace eeros::control {

/**
 * Reads N keys from a keyboard and delivers their state as bool output signals.
 *
 * @tparam N - number of keys
 *
 * @since v0.6
 */
template< uint8_t N >
class KeyboardInput: public Blockio<0,N,bool> {
 public:
  /**
   * Constructs a KeyboardInput block.
   * @param asciiCode - ascii code of the key
   * @param name - names of the key
   * @param priority - priority of the associated thread
   */
  KeyboardInput(std::vector<char> asciiCode, std::vector<std::string> name, int priority = 20) : k(priority) {
    auto& list = hal::KeyList::instance();
    list.addKeys(asciiCode, name);
    auto& hal = hal::HAL::instance();
    for (uint8_t i = 0; i < list.nofKeys; i++) {
      hal.addInput(new hal::KeyboardDigIn(list, list.key[i]));
    }
  }

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  KeyboardInput(const KeyboardInput& s) = delete;
  KeyboardInput& operator=(const KeyboardInput&) = delete;

  /**
   * Runs the block.
   */
  void run() override {
    uint64_t time = eeros::System::getTimeNs();
    auto& list = hal::KeyList::instance();
    for (uint8_t i = 0; i < list.nofKeys; i++) {
      this->out[i].getSignal().setValue(list.state[i]);
      this->out[i].getSignal().setTimestamp(time);
    }
  }

  /**
   * Returns the output at the given index.
   *
   * @param index - output index
   * @return the output with this index
   * @throws IndexOutOfBoundsFault if index >= N
   */
  auto& getOut(uint8_t index) requires (N > 1) {
    if (index >= N)
      throw IndexOutOfBoundsFault(
        "Trying to get inexistent element of output in block '" + this->getName() + "'");
    return this->out[index];
  }

  /**
   * Resets the state of the key at the given index.
   * Must be called manually after consuming the state.
   *
   * @param index - output index
   * @throws IndexOutOfBoundsFault if index >= N
   */
  virtual void reset(uint8_t index = 0) {
    if (index >= N)
      throw IndexOutOfBoundsFault(
        "Trying to get inexistent element of output in block '" + this->getName() + "'");
    hal::KeyList::instance().state[index] = false;
  }

 protected:
  hal::Keyboard k;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * KeyboardInput instance to an output stream.\n
 * Does not print a newline control character.
 */
template <uint8_t N>
std::ostream& operator<<(std::ostream& os, const KeyboardInput<N>& k) {
  os << "Block keyboard input: '" << k.getName() << "'";
  return os;
}

}

#endif /* ORG_EEROS_CONTROL_KEYBOARDINPUT_HPP_ */
