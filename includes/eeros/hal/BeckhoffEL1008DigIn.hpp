#ifndef ORG_EEROS_BECKHOFFEL1008_DIGIN_HPP_
#define ORG_EEROS_BECKHOFFEL1008_DIGIN_HPP_

#include <eeros/hal/Input.hpp>
#include <EcMasterlibMain.hpp>
#include <device/beckhoff/EL1008.hpp>
#include <string>

namespace eeros {
namespace hal {
    
/**
 * This class is part of the hardware abstraction layer. 
 * It is used to deliver one of the inputs of a Beckhoff digital 
 * input device as critical input to the safety system. Do not create instances of this class.
 *
 * @since v1.3
 */
class BeckhoffEL1008DigIn : public Input<bool> {
 public:
  /**
   * Constructs a digital input instance.
   * 
   * @param iface - reference to Beckhoff device
   * @param index - number of the digital signal (0 < index < 7)
   * @param id - name of the digital input
   */
  BeckhoffEL1008DigIn(ecmasterlib::device::beckhoff::EL1008& iface, int index, std::string id) 
      : iface(iface), index(index), Input<bool>(id, nullptr) { }
  
  /**
  * Disabling use of copy constructor because the block should never be copied unintentionally.
  */
  BeckhoffEL1008DigIn(const BeckhoffEL1008DigIn& s) = delete; 

  /**
   * Getter function for the state of this digital input
   * 
   * @return state of the digital input
   */
  virtual bool get() {
    return iface[index];
  }

 private:
  int index;
  ecmasterlib::device::beckhoff::EL1008& iface;
};

}
}

#endif /* ORG_EEROS_BECKHOFFEL1008_DIGIN_HPP_ */
