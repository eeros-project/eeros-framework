#ifndef ORG_EEROS_CONTROL_ODRIVEINPUT_USB_HPP
#define ORG_EEROS_CONTROL_ODRIVEINPUT_USB_HPP

#ifdef EEROS_USE_ODRIVE

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/hal/ODriveUSB.hpp>

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
class ODriveUSBInput : public Blockio<1,3,Vector2> {
 public:
  /**
   * Constructs a ODriveInput_USB instance \n
   * @param serialNr - odrive serial number
   * @param encTicks - encoder ticks of motors connected
   * @param priority - execution priority of the underlying thread
   * @param firstDrive - one drive must be set as the first one for reset tasks 
   */
  ODriveUSBInput(uint64_t serialNr, float encTicks, int priority, bool firstDrive) 
      : odrive(serialNr, encTicks, priority, firstDrive) { }
  
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  ODriveUSBInput(const ODriveUSBInput& s) = delete; 
  
  /**
   * Gets input data from ODrive_USB thread and outputs them
   */
  virtual void run() override {
    // Set speed
    odrive.setRefVel(0, in.getSignal().getValue()(0), 1/6.28); // rad/s
    odrive.setRefVel(1, in.getSignal().getValue()(1), 1/6.28); // rad/s
    // Output speed
    double vel0 = odrive.getEncoderVel(0, 1/6.28); // rad/s
    double vel1 = odrive.getEncoderVel(1, 1/6.28); // rad/s
    Vector2 vel; vel << vel0, vel1;
    out[0].getSignal().setValue(vel);
    // Output current
    double curr0 = odrive.getCurrentMeasured(0);
    double curr1 = odrive.getCurrentMeasured(1);
    Vector2 curr; curr << curr0, curr1;
    out[1].getSignal().setValue(curr);
    // Output current setpoint
    double setp0 = odrive.getCurrentSetpoint(0);
    double setp1 = odrive.getCurrentSetpoint(1);
    Vector2 setp; setp << setp0, setp1;
    out[2].getSignal().setValue(setp);
    // set timestamps
    uint64_t ts = eeros::System::getTimeNs();
    out[0].getSignal().setTimestamp(ts);
    out[1].getSignal().setTimestamp(ts);
    out[2].getSignal().setTimestamp(ts);
  }
  
  /**
   * Enables drives
   */
  virtual void enable() override {
    odrive.enableDrives();
  }
 
  /**
   * Disables drives
   */
  virtual void disable() override {
    odrive.disableDrives();
  }
  
  /**
   * Checks for drive emergency status
   * 
   * @return true, if odrive is in emergency 
   */
  virtual bool isEmergency() {
    return odrive.isEndstopActive();
  }
  
  /**
   * Checks for drive calibration status
   * 
   * @return true, if odrive is calibrated 
   */
  virtual bool isCalibrated() {
    return odrive.isCalibrated();
  }
  
  /**
   * Starts odrive calibration procedure
   */
  virtual void startCalibration() {
    odrive.startCalibration();
  }
  
 private:
  ODriveUSB odrive;
};

/**
 * Operator overload (<<) to enable an easy way to print the state of a
 * odrive instance to an output stream.\n
 * Does not print a newline control character.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, ODriveUSBInput& odrive) {
  os << "Block ODriveUSB input: '" << odrive.getName(); 
  return os;
}

}
}

#endif
#endif /* ORG_EEROS_CONTROL_ODRIVEINPUT_USB_HPP */
