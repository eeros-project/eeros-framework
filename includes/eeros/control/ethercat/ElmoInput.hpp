#ifndef ORG_EEROS_CONTROL_ETHERCAT_ELMOINPUT_
#define ORG_EEROS_CONTROL_ETHERCAT_ELMOINPUT_

#include <ecmasterlib/device/Elmo.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>

namespace eeros {
namespace control {

/**
 * This block reads a Elmo drive over EtherCAT and outputs the position,
 * velocity and torque values onto three output signals. All values must be scaled 
 * in order to get meaningful physical entities.
 *
 * @since v1.3
 */

class ElmoInput : public Blockio<0,0,double,double> {
  static constexpr ecmasterlib::Context ctx = ecmasterlib::Context::Cyclic;

 public:
  /**
   * Constructs a EtherCAT receive block instance which receives its output 
   * signals from a Elmo drive.
   *
   * @param iface - reference to Elmo drive
   */
  ElmoInput(ecmasterlib::Elmo iface) : iface(iface) {}

  /**
   * Puts the drive inputs onto the output signals.
   */
  virtual void run() {
    uint64_t ts = eeros::System::getTimeNs();
    position.getSignal().setValue(iface.getPosition(ctx));
    position.getSignal().setTimestamp(ts);
    velocity.getSignal().setValue(iface.getVelocity(ctx));
    velocity.getSignal().setTimestamp(ts);
    torque.getSignal().setValue(iface.getTorque(ctx));
    torque.getSignal().setTimestamp(ts);
  }
  
  /**
   * Gets the position output of the block.
   * 
   * @return output
   */
  virtual Output<int32_t>& getPosOut() { return position; }

  /**
   * Gets the velocity output of the block.
   * 
   * @return output
   */
  virtual Output<int32_t>& getVelOut() { return velocity; }

  /**
   * Gets the torque output of the block.
   * 
   * @return output
   */
  virtual Output<int16_t>& getTorqueOut() { return torque; }

  /**
   * Gets the current state of the elmo drive.
   * States are: SWITCH_ON_DISABLED, OPERATION_ENABLED, FAULT, etc.
   * 
   * @return state
   */
  virtual ecmasterlib::types::ds402::State getState() {
    return iface.getState(ctx);
  }

  /**
   * Gets the state description of the elmo drive.
   * 
   * @return state description
   */
  virtual const char* getStateDesc() {
    return iface.getState(ctx).stateToText();
  }

  /**
   * Gets the current mode of the elmo drive.
   * Modes are: HOMING, PROFILE_VELOCITY, etc.
   * 
   * @return state
   */
  virtual ecmasterlib::types::ds402::Mode getMode() {
    return iface.getMode(ctx);
  }

 private:
  ecmasterlib::Elmo iface;
  Output<int32_t> position, velocity;
  Output<int16_t> torque;
};

}
}

#endif // ORG_EEROS_CONTROL_ETHERCAT_ELMOINPUT_
