#ifndef ORG_EEROS_CONTROL_ETHERCAT_EL1008_
#define ORG_EEROS_CONTROL_ETHERCAT_EL1008_

#include <eeros/control/Blockio.hpp>
#include <eeros/core/System.hpp>
#include <EcMasterlibMain.hpp>
#include <device/beckhoff/EL1008.hpp>
#include <eeros/hal/HAL.hpp> 
#include <eeros/hal/Input.hpp>
#include <eeros/hal/BeckhoffEL1008DigIn.hpp>

using namespace eeros::hal;

namespace eeros {
namespace control {

/**
 * This block reads a Beckhoff digital input device EL1008 over EtherCAT 
 * and outputs the values onto up to 8 digital output signals. 
 * The first two input signals (index 0 and 1) of the EL1008 can be used 
 * as critical inputs to the safety system.
 *
 * @since v1.3
 */

class BeckhoffEL1008 : public Blockio<0,8,bool,bool> {
  
 public:
  /**
   * Constructs a EtherCAT receive block instance which receives its digital output 
   * signals from a Beckhoff digital input device.
   *
   * @param iface - reference to Beckhoff device
   */
  BeckhoffEL1008(ecmasterlib::device::beckhoff::EL1008& iface) : iface(iface) {
    HAL& hal = HAL::instance();
    hal::Input<bool>* in = new BeckhoffEL1008DigIn(iface, 0, "EL1008_0");
    hal.addInput(in);
    in = new BeckhoffEL1008DigIn(iface, 1, "EL1008_1");
    hal.addInput(in);
  }
          
  /**
   * Puts the drive inputs onto the digital output signals.
   */
  virtual void run() {
    int val = iface.getInputs();
    uint64_t ts = eeros::System::getTimeNs();
    for(int i = 0; i < 8; i++) {
      out[i].getSignal().setValue((val & (1 << i)) != 0);
      out[i].getSignal().setTimestamp(ts);
    }
  }
  
 private:
   ecmasterlib::device::beckhoff::EL1008& iface;
};

}
}

#endif // ORG_EEROS_CONTROL_ETHERCAT_EL1008_
