#ifndef ORG_EEROS_CONTROL_PERIPHERALINPUT_HPP
#define ORG_EEROS_CONTROL_PERIPHERALINPUT_HPP

#include <eeros/control/Blockio.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/core/Fault.hpp>

namespace eeros {
namespace control {

/**
 * A peripheral input block reads a signal from an input. This
 * input must be defined by the hardware configuration file.
 * 
 * @tparam T - output signal data type, must be bool or double (double - default type)
 * @tparam U - output signal unit type (dimensionless - default type)
 *
 * @since v0.4
 */

template < typename T = double, SIUnit U = SIUnit::create() >
requires std::is_same_v<T, double> || std::is_same_v<T, bool>
class PeripheralInput : public Blockio<0,1,T,T,siunit::generateNSizeArray<0>(),MakeUnitArray<U>::value> {
 public:
  /**
   * Constructs a peripheral input instance with a name defined in the 
   * hardware configuration file.
   *
   * @param id - name of the input
   * @param exclusive - if true, no other input can claim this input signal
   */
  PeripheralInput(std::string id, bool exclusive = true) : hal(hal::HAL::instance()) {
    systemInput = dynamic_cast<eeros::hal::Input<T>*>(hal.getInput(id, exclusive));
    if(systemInput == nullptr) throw Fault("Peripheral input '" + id + "' not found!");
    if(systemInput->getUnit() != U) throw Fault("Expected output signal unit type does not match unit type of " + id);
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  PeripheralInput(const PeripheralInput& s) = delete; 

  /**
   * Samples the signal at the input.
   */
  void run() override {
    this->out.getSignal().setValue(systemInput->get());
    this->out.getSignal().setTimestamp(systemInput->getTimestamp());
  }

  /**
   * Calls a feature function. This function allows to configure hardware specific features defined
   * in the hardware wrapper library.
   *
   * @tparam ArgTypesOut - type of the arguments
   * @param featureName - name of the feature function
   * @param args - argument list of variable size
   */
  template<typename ... ArgTypesIn>
  void callInputFeature(std::string featureName, ArgTypesIn... args){
    hal.callInputFeature(systemInput, featureName, args...);
  }

 private:
  hal::HAL& hal;
  hal::Input<T>* systemInput;
};

}
}

#endif // ORG_EEROS_CONTROL_PERIPHERALINPUT_HPP
