#ifndef ORG_EEROS_HAL_KEYLIST_HPP_
#define ORG_EEROS_HAL_KEYLIST_HPP_

#include <vector>
#include <string>

namespace eeros {
namespace hal {

/**
 * This class is part of the hardware abstraction layer. 
 * It holds registered keyboard keys, which in turn can be read by a control block
 * input or a critical input to the safety system.
 *
 * @since v1.2
 */

class KeyList {
 private:
  KeyList() {}
  
 public:
  /**
   * Returns an instance of a key list
   * 
   * @return instance of the list
   */
  static KeyList& instance() {
    static KeyList list;
    return list;
  }

  /**
   * Add keys with given ascii codes and names to the list.
   * Each key can be named.
   * 
   * @param asciiCode - ascii code of the keyboard
   * @param name - name of the key
   */
  void addKeys(std::vector<char> asciiCode, std::vector<std::string> name) {
    nofKeys = 0;
    for (auto k : asciiCode) {
      this->key.push_back(name[nofKeys]);
      this->asciiCode.push_back(k);
      this->state.push_back(false);
      this->event.push_back(false);
      nofKeys++;
    }
  }

  uint8_t nofKeys;
  std::vector<std::string> key;
  std::vector<char> asciiCode;
  std::vector<bool> state;
  std::vector<bool> event;
};

}
}

#endif //ORG_EEROS_HAL_KEYLIST_HPP_
