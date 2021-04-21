#ifndef ORG_EEROS_HAL_KEYBOARD_HPP_
#define ORG_EEROS_HAL_KEYBOARD_HPP_

#include <termios.h>
#include <eeros/core/Thread.hpp>

namespace eeros {
namespace hal {

/**
 * This class is part of the hardware abstraction layer. 
 * It is used by \ref eeros::control::KeyboardInput and \ref eeros::hal::KeyboardDigIn class. 
 * Do not use it directly.
 *
 * @since v0.6
 */
class Keyboard : public Thread {
 public:
  explicit Keyboard(int priority);
  ~Keyboard();
  
 private:
  virtual void run();
  bool running;
  struct termios tio;
};

}
}

#endif // ORG_EEROS_HAL_KEYBOARD_HPP_
