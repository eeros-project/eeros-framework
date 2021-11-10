#ifndef ORG_EEROS_CONTROL_ODRIVEINPUT_UART_HPP
#define ORG_EEROS_CONTROL_ODRIVEINPUT_UART_HPP

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/hal/ODriveUART.hpp>

using namespace eeros::math;
using namespace eeros::hal;

namespace eeros {
namespace control {

/**
 * This block controls an ODrive motor controller which can control two motors.
 * The block offers 1 input for velocity control values and three outputs for
 * actual velocity, actual current and current setpoint.
 *
 * @since v1.3
 */
class ODriveUARTInput : public Blockio<1,3,Vector2> {
 public:
  /**
   * Constructs a ODriveUARTInput instance \n
   * @param dev - string with device id
   * @param speed - communication speed of UART
   * @param parity - parity bit set or not
   * @param priority - execution priority of this thread
   */
  ODriveUARTInput(std::string dev, int speed, int parity, int priority): odrive(dev, speed, parity, priority) { }
  
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  ODriveUARTInput(const ODriveUARTInput& s) = delete; 
  
  /**
   * Gets input data from ODriveUART thread and outputs them
   */
  virtual void run() {
    // Output data 
    double vel0 = odrive.getVel(0);
    double vel1 = odrive.getVel(1);
    Vector2 vel; vel << vel0, vel1;
    out[0].getSignal().setValue(vel);
    // Timestamps 
//    uint64_t ts = eeros::System::getTimeNs();
  }
  
 private:
  ODriveUART odrive;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * odrive instance to an output stream.\n
 * Does not print a newline control character.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, ODriveUARTInput& odrive) {
  os << "Block ODriveUART input: '" << odrive.getName(); 
  return os;
}

}
}

#endif /* ORG_EEROS_CONTROL_ODRIVEINPUT_UART_HPP */
