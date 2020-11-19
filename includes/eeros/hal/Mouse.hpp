#ifndef ORG_EEROS_HAL_MOUSE_HPP_
#define ORG_EEROS_HAL_MOUSE_HPP_

#include <string>
#include <functional>
#include <linux/input.h>
#include <eeros/hal/Input.hpp>
#include <eeros/core/Thread.hpp>

#define MOUSE_BUTTON_COUNT (16)
#define MOUSE_AXIS_COUNT (8)

namespace eeros {
namespace hal {

struct MouseState {
  struct {
    bool left;
    bool middle;
    bool right;
  } button;
  struct {
    signed x;
    signed y;
    signed z;
    signed r;
  } axis;
};

/**
 * This class is part of the hardware abstraction layer. It is used by a \ref eeros::control::MouseInput class. 
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
  
  MouseState current;
  MouseState last;

 private:
  virtual void run();
  virtual bool open(const char* device);
  virtual void close();
  int fd;
  bool running;
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
