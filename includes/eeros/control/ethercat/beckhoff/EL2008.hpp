#ifndef ORG_EEROS_CONTROL_ETHERCAT_EL2008_
#define ORG_EEROS_CONTROL_ETHERCAT_EL2008_

#include <eeros/control/Blockio.hpp>
#include <EcMasterlibMain.hpp>
#include <device/beckhoff/EL2008.hpp>

namespace eeros {
namespace control {

/**
 * This block reads up to 8 digital input signals and sends them over EtherCAT 
 * to a Beckhoff digital output device EL2008. 
 *
 * @since v1.3
 */

class BeckhoffEL2008 : public Blockio<8,0,bool,bool> {
  
 public:
  /**
   * Constructs a EtherCAT send block instance which sends its digital input 
   * signals to a Beckhoff digital output device.
   *
   * @param iface - reference to Beckhoff device
   */
  BeckhoffEL2008(ecmasterlib::device::beckhoff::EL2008& iface) : iface(iface) { }
          
  /**
   * Puts the digital input signals onto the drive outputs.
   */
  virtual void run() {
    int val;
    for(int i = 0; i < 8; i++) {
      bool state = in[i].getSignal().getValue(); 
      if(state) val |= (1 << i);
      else val &= ~(1 << i); 
    }
    iface.setAll(val);             
  }
  
 private:
   ecmasterlib::device::beckhoff::EL2008& iface;
};

}
}

#endif // ORG_EEROS_CONTROL_ETHERCAT_EL2008_
