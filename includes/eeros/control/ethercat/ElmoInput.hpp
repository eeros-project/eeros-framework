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
    position.getSignal().setValue(elmo.getPosition());
    position.getSignal().setTimestamp(timestamp);
    velocity.getSignal().setValue(elmo.getVelocity());
    velocity.getSignal().setTimestamp(timestamp);
    torque.getSignal().setValue(elmo.getTorque());
    torque.getSignal().setTimestamp(timestamp);
  }
  
  /**
   * Gets the position output of the block.
   * 
   * @return output
   */
  virtual Output<int32_t>& getPosOut(){ return position; }

  /**
   * Gets the velocity output of the block.
   * 
   * @return output
   */
  virtual Output<int32_t>& getVelOut(){ return velocity; }

  /**
   * Gets the torque output of the block.
   * 
   * @return output
   */
  virtual Output<int16_t>& getTorqueOut(){ return torque; }

 private:
   ecmasterlib::device::Elmo& iface;
   Output<int32_t> position, velocity;
   Output<int16_t> torque;
};

}
}

#endif // ORG_EEROS_CONTROL_ETHERCAT_ELMOINPUT_
