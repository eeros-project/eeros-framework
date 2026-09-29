#ifndef ORG_EEROS_CONTROL_ETHERCAT_ELMOOUTPUT_
#define ORG_EEROS_CONTROL_ETHERCAT_ELMOOUTPUT_

#include <cstdint>
#include <ecmasterlib/device/Elmo.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/core/System.hpp>

#include "ecmasterlib/device/Device.hpp"
#include "ecmasterlib/device/od/DS402.hpp"

namespace eeros {
namespace control {

/**
 * This block reads input signals for position, velocity and torque and sends
 * them over EtherCAT to an Elmo drive. All values must be scaled from physical 
 * entities to fit into the transmit frame of EtherCAT.
 *
 * @since v1.3
 */

class ElmoOutput : public Blockio<0,0,double,double> {
  static constexpr ecmasterlib::Context ctx = ecmasterlib::Context::Cyclic;

 public:
  /**
   * Constructs an EtherCAT transmit block instance which sends its input 
   * signals to an Elmo drive.
   *
   * @param iface - reference to Elmo drive
   */
  ElmoOutput(ecmasterlib::Elmo iface) : iface(iface) {}

  /**
   * Puts the signal inputs to the Elmo drive.
   */
  virtual void run() {
    using Mode = ecmasterlib::types::ds402::Mode;
    const auto getValue = [](auto &input) { return input.getSignal().getValue(); };
    switch (iface.getMode(ctx)) {
      case Mode::PROFILE_POSITION:
        iface.setTargetPosition(ctx, getValue(position));
        iface.setMaximumTorque(ctx, getValue(torqueMax));
        break;
      case Mode::PROFILE_VELOCITY:
        iface.setTargetVelocity(ctx, getValue(velocity));
        iface.setMaximumTorque(ctx, getValue(torqueMax));
        break;
      case Mode::PROFILE_TORQUE:
        iface.setTargetTorque(ctx, getValue(torque));
        iface.setMaximumTorque(ctx, getValue(torqueMax));
        break; 
      default:
        break;
    }
  }
  
  /**
   * Gets the position input of the block.
   * 
   * @return input
   */
  virtual Input<int32_t>& getPosIn() { return position; } 
  
  /**
   * Gets the velocity input of the block.
   * 
   * @return input
   */
  virtual Input<int32_t>& getVelIn() { return velocity; }
  
  /**
   * Gets the torque input of the block.
   * 
   * @return input
   */
  virtual Input<int16_t>& getTorqueIn() { return torque; }
  
  /**
   * Gets the maximum torque input of the block.
   * 
   * @return input
   */
  virtual Input<uint16_t>& getTorqueMaxIn() { return torqueMax; }

  /**
   * Sets the mode of the elmo drive.
   * Modes are: HOMING, PROFILE_VELOCITY, etc.
   * 
   * @param mode mode of the drive
   */
  virtual void setMode(ecmasterlib::types::ds402::Mode mode) {
    iface.setMode(ctx, mode);
  }

  /**
   * Sends a command to the elmo drive.
   * Commands are: SWITCH_ON, ENABLE_OPERATION, etc.
   * 
   * @param cmd mode of the drive
   */
  virtual void sendCommand(ecmasterlib::Command cmd) {
    iface.sendCommand(ctx, cmd);
  }

 private:
  ecmasterlib::Elmo iface;
  Input<int32_t> position, velocity;
  Input<int16_t> torque;
  Input<uint16_t> torqueMax;
};

}
}

#endif // ORG_EEROS_CONTROL_ETHERCAT_ELMOOUTPUT_
