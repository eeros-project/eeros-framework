#include <eeros/hal/Keyboard.hpp>
#include <eeros/hal/KeyList.hpp>
#include <unistd.h>
#include <fcntl.h>

using namespace eeros::hal;

Keyboard::Keyboard(int priority) : Thread(priority) {
  tcgetattr(STDIN_FILENO, &tio);
  tio.c_lflag &=(~ICANON & ~ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &tio);
}

Keyboard::~Keyboard() {     
  running = false; 
  join(); 
  tio.c_lflag |=(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &tio);
}

void Keyboard::run() {
  running = true;
  auto& list = KeyList::instance();
  while (running) {
    char c = std::cin.get();
    for (uint8_t i = 0; i < list.nofKeys; i++) {
      if (c == list.asciiCode[i]) {
        list.state[i] = true;
        list.event[i] = true;
      }
    }
    usleep(10000);
  }
}
