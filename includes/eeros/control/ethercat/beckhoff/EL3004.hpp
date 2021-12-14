#ifndef ORG_EEROS_CONTROL_ETHERCAT_EL3004_
#define ORG_EEROS_CONTROL_ETHERCAT_EL3004_

#include <eeros/control/Blockio.hpp>
#include <eeros/core/System.hpp>
#include <EcMasterlibMain.hpp>
#include <device/beckhoff/EL3004.hpp>

namespace eeros {
namespace control {

/**
 * This block reads a Beckhoff analog input device EL3004 over EtherCAT 
 * and outputs the values onto up to 8 analog output signals. 
 * EtherCAT delivers values of type int16_t. These values cover the range 
 * between -10V and +10V.
 *
 * @since v1.3
 */

class BeckhoffEL3004 : public Blockio<0,4,double,double> {
  
 public:
  /**
   * Constructs a EtherCAT receive block instance which receives its analog output 
   * signals from a Beckhoff analog input device.
   *
   * @param iface - reference to Beckhoff device
   */
  BeckhoffEL3004(ecmasterlib::device::beckhoff::EL3004& iface) : iface(iface) { }
          
  /**
   * Puts the drive inputs onto the analog output signals.
   */
  virtual void run() {
    uint64_t ts = eeros::System::getTimeNs();
    for(int i = 0; i < 4; i++) {
      out[i].getSignal().setValue((double)iface[i] / 3276.8);
      out[i].getSignal().setTimestamp(ts);
    }
  }
  
 private:
   ecmasterlib::device::beckhoff::EL3004& iface;
};

}
}

#endif // ORG_EEROS_CONTROL_ETHERCAT_EL3004_
