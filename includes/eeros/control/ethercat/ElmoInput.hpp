#ifndef ORG_EEROS_CONTROL_ETHERCAT_ELMOINPUT_
#define ORG_EEROS_CONTROL_ETHERCAT_ELMOINPUT_

#include <eeros/control/Blockio.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/core/System.hpp>
#include <EcMasterlibMain.hpp>
#include <device/Elmo.hpp>

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
  
 public:
  /**
   * Constructs a EtherCAT receive block instance which receives its output 
   * signals from a Elmo drive.
   *
   * @param iface - reference to Elmo drive
   */
  ElmoInput(ecmasterlib::device::Elmo& iface) : iface(iface) { }
          
  /**
   * Puts the drive inputs onto the output signals.
   */
  virtual void run() {
    uint64_t ts = eeros::System::getTimeNs();
    position.getSignal().setValue(iface.getPosition());
    position.getSignal().setTimestamp(ts);
    velocity.getSignal().setValue(iface.getVelocity());
    velocity.getSignal().setTimestamp(ts);
    torque.getSignal().setValue(iface.getTorque());
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
  virtual ecmasterlib::device::Elmo::State getState() { return iface.getState(); }

  /**
   * Gets the state description of the elmo drive.
   * 
   * @return state description
   */
  virtual const char* getStateDesc() { return iface.stateToText(iface.getState()); }
  
  /**
   * Gets the current mode of the elmo drive.
   * Modes are: HOMING, PROFILE_VELOCITY, etc.
   * 
   * @return state
   */
  virtual ecmasterlib::device::Elmo::Mode getMode() { return iface.getMode(); }

 private:
  ecmasterlib::device::Elmo& iface;
  Output<int32_t> position, velocity;
  Output<int16_t> torque;
};

}
}

#endif // ORG_EEROS_CONTROL_ETHERCAT_ELMOINPUT_
