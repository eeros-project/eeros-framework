#ifndef ORG_EEROS_CONTROL_ETHERCAT_EL4004_
#define ORG_EEROS_CONTROL_ETHERCAT_EL4004_

#include <eeros/control/Blockio.hpp>
#include <EcMasterlibMain.hpp>
#include <device/beckhoff/EL4004.hpp>

namespace eeros {
namespace control {

/**
 * This block reads up to 4 analog input signals and sends them over EtherCAT 
 * to a Beckhoff analog output device EL4004. 
 * EtherCAT sends values of type int16_t. These values cover the range 
 * between 0V and +10V.
 *
 * @since v1.3
 */

class BeckhoffEL4004 : public Blockio<4,0,double,double> {
  
 public:
  /**
   * Constructs a EtherCAT send block instance which sends its analog input 
   * signals to a Beckhoff analog output device.
   *
   * @param iface - reference to Beckhoff device
   */
  BeckhoffEL4004(ecmasterlib::device::beckhoff::EL4004& iface) : iface(iface) { }
          
  /**
   * Puts the analog input signals onto the drive outputs.
   */
  virtual void run() {
    for(int i = 0; i < 4; i++) {
      double val = in[i].getSignal().getValue(); 
      if (val < 0.0 ) val = 0.0;   // not lower than  0V 
      if (val > 10.0 ) val = 10.0; // not higher than 10V
      iface.setChannel(i, (int16_t)(val * 3276.7));             
    }
  }
  
 private:
   ecmasterlib::device::beckhoff::EL4004& iface;
};

}
}

#endif // ORG_EEROS_CONTROL_ETHERCAT_EL4004_
