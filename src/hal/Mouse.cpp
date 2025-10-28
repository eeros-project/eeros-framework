#include <eeros/hal/Mouse.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/MouseDigIn.hpp>
#include <eeros/core/Fault.hpp>

#include <fcntl.h>
#include <unistd.h>

using namespace eeros::hal;

Mouse::Mouse(std::string dev, int priority) : Thread(priority) {
  open(dev.c_str());
  left = new MouseDigIn("leftMouseButton", this);
  middle = new MouseDigIn("middleMouseButton", this);
  right = new MouseDigIn("rightMouseButton", this);
  HAL& hal = HAL::instance();
  hal.addInput(left);
  hal.addInput(middle);
  hal.addInput(right);
  running.store(true, std::memory_order_release);
}

Mouse::~Mouse() {
  running.store(false, std::memory_order_relaxed);
  join();
  close();
}

bool Mouse::open(const char* device) {
  fd = ::open(device, O_RDONLY | O_NONBLOCK);
  if (fd < 0) log.error() << "Mouse: could not open input device on " + std::string(device);
  return fd;
}

void Mouse::close() {
        ::close(fd);
}

std::string Mouse::name() {
  if (!fd) return "";
  char name[128];
  if (ioctl(fd, EVIOCGNAME (sizeof(name)), name)) {
    name[127] = 0;
    return name;
  } else return "";
}

void Mouse::on_event(std::function<void(struct input_event)> action) {
        event_action = action;
}

void Mouse::on_button(std::function<void(int, bool)> action) {
        button_action = action;
}

void Mouse::on_axis(std::function<void(int, signed)> action) {
        axis_action = action;
}


void Mouse::run() {
  // synchronize with constructor
  // this is vital to read from the actual fd and not whatever happened to be
  // stored in the unitialized fd
  while(!running.load(std::memory_order_acquire)) usleep(1000);
  if (fd < 0) {
    log.error() << "failed to open mouse file descriptor";
    return;
  }
  struct input_event e;
  eeros::hal::MouseState currentState{};
  eeros::hal::MouseState lastState{};
  while (running.load(std::memory_order_relaxed)) {
    ssize_t n = read(fd, &e, sizeof(struct input_event));
    if (n > 0) {
      if (e.type == EV_KEY) {
        switch (e.code) {
          case BTN_LEFT: currentState.button.left = e.value; break;
          case BTN_MIDDLE: currentState.button.middle = e.value; break;
          case BTN_RIGHT: currentState.button.right = e.value; break;
          default: break;
        }

        if (button_action != nullptr) button_action(e.code, e.value);
      } else if (e.type == EV_REL) {
        switch (e.code) {
          case REL_X: currentState.axis.x += e.value; break;
          case REL_Y: currentState.axis.y += e.value; break;
          case REL_WHEEL: currentState.axis.z += e.value; break;
          case REL_HWHEEL: currentState.axis.r += e.value; break;
          default: break;
        }

        if (axis_action != nullptr) axis_action(e.code, e.value);
      }

      if (event_action != nullptr) event_action(e);

      current.write(currentState);
      last.write(lastState);
      lastState = currentState;
    } else usleep(1000);
  }
}
