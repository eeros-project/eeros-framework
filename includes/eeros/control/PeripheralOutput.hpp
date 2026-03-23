#ifndef ORG_EEROS_CONTROL_PERIPHERALOUTPUT_HPP
#define ORG_EEROS_CONTROL_PERIPHERALOUTPUT_HPP

#include <cmath>
#include <mutex>
#include <eeros/control/Blockio.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/control/NaNOutputFault.hpp>

namespace eeros {
namespace control {

/**
 * A peripheral output block delivers a signal to an output. This
 * output must be defined by the hardware configuration file.
 * 
 * @tparam T - input signal data type, must be bool or double (double - default type)
 * @tparam U - output signal unit type (dimensionless - default type)
 *
 * @since v0.4
 */

template < typename T = double, SIUnit U = SIUnit::create() >
requires std::is_same_v<T, double> || std::is_same_v<T, bool>
class PeripheralOutput : public Blockio<1,0,T,T,MakeUnitArray<U>::value> {
 public:
  /**
   * Constructs a peripheral output instance with a name defined in the 
   * hardware configuration file.
   *
   * @param id - name of the input
   * @param exclusive - if true, no other output can claim this output signal
   */
  PeripheralOutput(std::string id, bool exclusive = true) : hal(hal::HAL::instance()) {
    systemOutput = dynamic_cast<hal::Output<T>*>(hal.getOutput(id, exclusive));
    if(systemOutput == nullptr) throw Fault("Peripheral output '" + id + "' not found!");
    if(systemOutput->getUnit() != U) throw Fault("Expected input signal unit type does not match unit type of " + id);
  }

  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  PeripheralOutput(const PeripheralOutput& s) = delete; 

  /**
   * Delivers the signal to the output.
   */
  void run() override {
    std::lock_guard<std::mutex> lock(mtx);
    val = this->in.getSignal().getValue();
    auto isSafe = false;
    if(std::isnan(val) || std::isinf(val)) {
      val = systemOutput->safe;
      isSafe = true;
    }
    systemOutput->set(val);
    systemOutput->setTimestampSignalIn(this->in.getSignal().getTimestamp());
    if (isSafe) throw NaNOutputFault("NaN written to output '" + 
                                     this->getName() + "', set to safe level if safe level is defined");
  }
  
  /**
   * Getter function for the signal value which the block delivers to its output.
   * 
   * @return - signal value
   */
  virtual T getValue () {
    std::lock_guard<std::mutex> lock(mtx);
    return val;
  }

  /**
   * Calls a feature function. This function allows to configure hardware specific features defined
   * in the hardware wrapper library.
   *
   * @tparam ArgTypesOut - type of the arguments
   * @param featureName - name of the feature function
   * @param args - argument list of variable size
   */
  template<typename ... ArgTypesOut>
  void callOutputFeature(std::string featureName, ArgTypesOut... args){
    hal.callOutputFeature(systemOutput, featureName, args...);
  }
 
 private:
  hal::HAL& hal;
  hal::Output<T>* systemOutput;
  T val;
  std::mutex mtx;
};

};
};

#endif // ORG_EEROS_CONTROL_PERIPHERALOUTPUT_HPP
