#ifndef ORG_EEROS_CONTROL_ETHERCAT_ELMOOUTPUT_
#define ORG_EEROS_CONTROL_ETHERCAT_ELMOOUTPUT_

#include <eeros/control/Blockio.hpp>
#include <eeros/control/Input.hpp>
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

class ElmoOutput : public Blockio<0,0,double,double> {
  
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
    using Mode = ecmasterlib::device::Elmo::Mode;
    const auto getValue = [](auto &input) { return input.getSignal().getValue(); };
    switch (elmo.getMode()) {
      case Mode::PROFILE_POSITION: 
        elmo.setTargetPosition(getValue(position));
        elmo.setMaximumTorque(getValue(torqueMax)); 
        break;
      case Mode::PROFILE_VELOCITY:
        elmo.setTargetVelocity(getValue(velocity));
        elmo.setMaximumTorque(getValue(torqueMax)); 
        break;
      case Mode::PROFILE_TORQUE: 
        elmo.setTargetTorque(getValue(torque));
        elmo.setMaximumTorque(getValue(torqueMax)); 
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
  virtual Input<int16_t>& getTorqueMaxIn(){ return torqueMax; }

 private:
   ecmasterlib::device::Elmo& iface;
   Input<int32_t> position, velocity;
   Input<int16_t> torque, torqueMax;
};

}
}

#endif // ORG_EEROS_CONTROL_ETHERCAT_ELMOOUTPUT_
