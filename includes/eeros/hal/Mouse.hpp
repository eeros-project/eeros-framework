#ifndef ORG_EEROS_HAL_MOUSE_HPP_
#define ORG_EEROS_HAL_MOUSE_HPP_

#include <string>
#include <functional>
#include <atomic>
#include <linux/input.h>
#include <eeros/hal/Input.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/core/AsyncBuffer.hpp>

#define MOUSE_BUTTON_COUNT (16)
#define MOUSE_AXIS_COUNT (8)

namespace eeros {
namespace hal {

struct MouseState {
  struct {
    bool left = false;
    bool middle = false;
    bool right = false;
  } button;
  struct {
    signed x = 0;
    signed y = 0;
    signed z = 0;
    signed r = 0;
  } axis;
};

/**
 * This class is part of the hardware abstraction layer. 
 * It is used by \ref eeros::control::MouseInput and \ref eeros::hal::MouseDigIn class. 
 * Do not use it directly.
 *
 * @since v0.6
 */
class Mouse : public eeros::Thread {
 public:
  explicit Mouse(std::string dev, int priority);
  ~Mouse();
  virtual void on_event(std::function<void(struct input_event)> action);
  virtual void on_button(std::function<void(int, bool)> action);
  virtual void on_axis(std::function<void(int, signed)> action);
  virtual std::string name();
  
  eeros::AsyncBuffer<MouseState> current;
  eeros::AsyncBuffer<MouseState> last;

 private:
  virtual void run();
  virtual bool open(const char* device);
  virtual void close();
  int fd;
  std::atomic<bool> running{false};
  std::function<void(struct input_event)> event_action;
  std::function<void(int, bool)> button_action;
  std::function<void(int, signed)> axis_action;
  Input<bool>* left;
  Input<bool>* middle;
  Input<bool>* right;
};

}
}

#endif // ORG_EEROS_HAL_MOUSE_HPP_
