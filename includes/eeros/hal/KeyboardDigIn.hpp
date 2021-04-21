#ifndef ORG_EEROS_KEYBOARD_DIGIN_HPP_
#define ORG_EEROS_KEYBOARD_DIGIN_HPP_

#include <eeros/hal/Input.hpp>
#include <eeros/hal/KeyList.hpp>
#include <string>

namespace eeros {
namespace hal {
    
/**
 * This class is part of the hardware abstraction layer. 
 * It is used to deliver one of the keys of the keyboard as critical inputs 
 * to the safety system. Do not create instances of this class.
 * It realizes an digital input block whose input state can be set by a given keyboard
 * being pressed on the keyboard. As soon as the input is read through its getter 
 * function, the state will be reset.
 *
 * @since v0.6v
 */
class KeyboardDigIn : public Input<bool> {
 public:
  /**
   * Constructs a digital input instance.
   * 
   * @param list - list with all registered keys
   * @param id - name of the key
   */
  KeyboardDigIn(KeyList& list, std::string id) : Input<bool>(id, nullptr), list(list) { }
  
  /**
  * Disabling use of copy constructor because the block should never be copied unintentionally.
  */
  KeyboardDigIn(const KeyboardDigIn& s) = delete; 

  /**
   * Getter function for the state of this digital input
   * 
   * @return state of the digital input
   */
  virtual bool get() {
    for (uint8_t i = 0; i < list.nofKeys; i++) {
      if (list.key[i] == getId()) {
        bool val = list.event[i];  
        list.event[i] = false;
        return val;
      }
    }
    return false;
  }

 private:
  KeyList& list;
};

}
}

#endif /* ORG_EEROS_KEYBOARD_DIGIN_HPP_ */
